// ============================================================================
//  radar_vision_tracker.ino
//  ESP32 + HLK-LD2450 mmWave radar + Seeed Grove Vision AI V2 (+ Pi camera)
//  + pan servo.  Fuses radar and on-device person detection into up to 3
//  tracks, pans the sensor head to keep the chosen person centred, and serves
//  a live dashboard over WiFi (no laptop needed).
//
//  Libraries (Arduino Library Manager):
//    - Seeed_Arduino_SSCMA   (pulls in ArduinoJson)
//    - ESP32Servo
//  Board: "XIAO_ESP32S3" or "ESP32 Dev Module" (esp32 core 2.x or 3.x)
// ============================================================================
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <Seeed_Arduino_SSCMA.h>
#include "mbedtls/base64.h"

#include "config.h"
#include "ld2450.h"
#include "dashboard.h"
#include "car_link.h"

// ---------------------------------------------------------------------------
//  Shared state
// ---------------------------------------------------------------------------
struct VisionDet {
  float cx, cy, w, h;   // normalised 0..1 in the camera frame
  uint8_t score;
};

struct VisionFrame {
  VisionDet det[8];
  uint8_t n = 0;
  float panAtCapture = 0;  // head angle when the frame was taken
  uint32_t ms = 0;         // capture time
  uint32_t seq = 0;
};

enum TrackSrc : uint8_t { SRC_NONE = 0, SRC_RADAR = 1, SRC_CAM = 2, SRC_BOTH = 3 };

struct Track {
  bool active = false;
  uint8_t id = 0;
  float worldBrg = 0;      // bearing relative to the servo base (deg, + = right)
  float rangeMm = 0;
  bool rangeEstimated = false;
  int16_t speed = 0;
  uint32_t lastRadar = 0, lastCam = 0, born = 0;
  float camWorldBrg = 0, radarWorldBrg = 0;
  VisionDet box{};
};

static LD2450 radar;
static SSCMA AI;
static Servo servo;
static WebServer server(80);

static portMUX_TYPE visMux = portMUX_INITIALIZER_UNLOCKED;
static VisionFrame visShared;             // written by vision task (core 0)
static volatile bool visionAlive = false;
static volatile uint32_t visionLastOk = 0;
static volatile uint16_t frameW = 240, frameH = 240;

static SemaphoreHandle_t jpegMutex;
static uint8_t *jpegFront = nullptr, *jpegBack = nullptr, *jpegSend = nullptr;
static size_t jpegFrontLen = 0;
static uint32_t jpegSeq = 0;
static const size_t JPEG_MAX = 32 * 1024;

static volatile float panDeg = 0;         // current commanded head angle (deg from centre)
static Track tracks[MAX_TRACKS];
static uint8_t nextId = 1;
static int lockedId = -1;
static uint32_t lastTargetMs = 0;

enum Mode { MODE_AUTO, MODE_HOLD, MODE_MANUAL };
static Mode mode = MODE_AUTO;
static float manualPan = 0;
static bool sweepOn = SWEEP_ENABLED;
static bool sweeping = false;
static int sweepDir = 1;

// RC car follow state
enum CarState { CAR_OFF, CAR_IDLE, CAR_TURN, CAR_BACKTURN, CAR_APPROACH, CAR_HOLD };
static const char *CAR_STATE_NAME[] = {"off", "idle", "turning", "backing", "approaching", "holding"};
static bool carFollow = false;        // off at boot; enabled from the dashboard
static bool carStopPending = false;
static CarState carState = CAR_OFF;
static bool carAligned = false;
static int carThrottle = 0, carSteer = 0;
static bool apMode = false;

// ---------------------------------------------------------------------------
//  Helpers
// ---------------------------------------------------------------------------
static inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
static inline float freshness(uint32_t now, uint32_t t, float tau) {
  if (!t) return 0;
  return expf(-(float)(now - t) / tau);
}

static void writeServo(float pan) {
  float a = SERVO_CENTER_DEG + (SERVO_INVERT ? -pan : pan);
  a = clampf(a, 0, 180);
  servo.writeMicroseconds(SERVO_MIN_US + (int)((SERVO_MAX_US - SERVO_MIN_US) * a / 180.0f));
}

// Read width/height out of a JPEG's SOF marker.
static bool jpegSize(const uint8_t *d, size_t n, uint16_t &w, uint16_t &h) {
  for (size_t i = 2; i + 9 < n; i++) {
    if (d[i] == 0xFF && (d[i + 1] == 0xC0 || d[i + 1] == 0xC2)) {
      h = (d[i + 5] << 8) | d[i + 6];
      w = (d[i + 7] << 8) | d[i + 8];
      return w && h;
    }
  }
  return false;
}

// ---------------------------------------------------------------------------
//  Vision task — runs on core 0 so slow I2C transfers never stall the servo
// ---------------------------------------------------------------------------
static void visionTask(void *) {
  uint32_t count = 0, fails = 0;
  for (;;) {
    if (!visionAlive) {
      if (AI.begin(&Wire)) {
        visionAlive = true;
        fails = 0;
        Serial.println("[vision] Grove Vision AI V2 connected");
      } else {
        vTaskDelay(pdMS_TO_TICKS(1000));
        continue;
      }
    }

    count++;
    bool wantImg = VISION_IMAGE_EVERY > 0 && (count % VISION_IMAGE_EVERY) == 0;
    float panCap = panDeg;
    uint32_t tCap = millis();
    // SSCMA: invoke(times, filter, show). filter=true -> results only (fast),
    // filter=false -> results + JPEG.
    int rc = wantImg ? AI.invoke(1, false, true) : AI.invoke(1, true, false);
    if (rc != CMD_OK) {
      if (++fails > 5) { visionAlive = false; Serial.println("[vision] lost, retrying"); }
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }
    fails = 0;
    visionLastOk = millis();

    if (wantImg) {
      String img = AI.last_image();
      size_t olen = 0;
      if (img.length() && mbedtls_base64_decode(jpegBack, JPEG_MAX, &olen,
                                                (const unsigned char *)img.c_str(), img.length()) == 0) {
        uint16_t w, h;
        if (jpegSize(jpegBack, olen, w, h)) { frameW = w; frameH = h; }
        xSemaphoreTake(jpegMutex, portMAX_DELAY);
        uint8_t *t = jpegFront; jpegFront = jpegBack; jpegBack = t;
        jpegFrontLen = olen;
        jpegSeq++;
        xSemaphoreGive(jpegMutex);
      }
    }

    VisionFrame f;
    float fw = frameW, fh = frameH;
    for (auto &b : AI.boxes()) {
      if (f.n >= 8) break;
      if (b.target != VISION_PERSON_CLASS || b.score < VISION_MIN_SCORE) continue;
      f.det[f.n++] = {b.x / fw, b.y / fh, b.w / fw, b.h / fh, b.score};
    }
    f.panAtCapture = panCap;
    f.ms = tCap;
    portENTER_CRITICAL(&visMux);
    f.seq = visShared.seq + 1;
    visShared = f;
    portEXIT_CRITICAL(&visMux);
    vTaskDelay(1);
  }
}

// ---------------------------------------------------------------------------
//  Fusion + tracking (runs at 50 Hz on core 1)
// ---------------------------------------------------------------------------
struct Det {
  bool radar = false, cam = false, estimated = false;
  float worldBrg = 0, radarWorld = 0, camWorld = 0, rangeMm = 0;
  int16_t speed = 0;
  uint32_t tRadar = 0, tCam = 0;
  VisionDet box{};
};

static float camBearing(const VisionDet &d) { return CAM_X_SIGN * (d.cx - 0.5f) * CAM_HFOV_DEG; }

static void fuse(uint32_t now) {
  Det dets[3 + 8];
  int nd = 0;

  // Radar detections (sensor frame -> world frame using current head angle)
  bool radarFresh = radar.lastFrameMs && (now - radar.lastFrameMs) < 400;
  if (radarFresh) {
    for (auto &t : radar.targets) {
      if (!t.valid) continue;
      float x = RADAR_X_SIGN * t.x_mm, y = t.y_mm;
      float r = sqrtf(x * x + y * y);
      if (r > RADAR_MAX_RANGE_MM) continue;
      Det &d = dets[nd++];
      d.radar = true;
      d.radarWorld = d.worldBrg = panDeg + atan2f(x, y) * RAD_TO_DEG;
      d.rangeMm = r;
      d.speed = t.speed_cms;
      d.tRadar = radar.lastFrameMs;
    }
  }
  int nRadar = nd;

  // Vision detections (world frame using the head angle at capture time)
  VisionFrame vf;
  portENTER_CRITICAL(&visMux);
  vf = visShared;
  portEXIT_CRITICAL(&visMux);
  if (vf.ms && (now - vf.ms) < VISION_STALE_MS) {
    bool used[3] = {false, false, false};
    for (int i = 0; i < vf.n; i++) {
      float w = vf.panAtCapture + camBearing(vf.det[i]);
      int best = -1; float bestErr = MATCH_DEG;
      for (int j = 0; j < nRadar; j++) {
        float e = fabsf(dets[j].radarWorld - w);
        if (!used[j] && e < bestErr) { bestErr = e; best = j; }
      }
      Det *d;
      if (best >= 0) {
        used[best] = true;
        d = &dets[best];
      } else {
        d = &dets[nd++];
        // No radar range: estimate from box height vs. a typical person height
        float ang = vf.det[i].h * CAM_VFOV_DEG * DEG_TO_RAD;
        d->rangeMm = clampf(PERSON_HEIGHT_M * 1000.0f / (2.0f * tanf(ang / 2)), 300, 8000);
        d->estimated = true;
      }
      d->cam = true;
      d->camWorld = w;
      d->worldBrg = w;  // camera bearing is more precise than radar
      d->tCam = vf.ms;
      d->box = vf.det[i];
    }
  }

  // Associate detections with existing tracks (greedy nearest)
  bool taken[MAX_TRACKS] = {false};
  bool assigned[11] = {false};
  for (int pass = 0; pass < nd; pass++) {
    float bestCost = 1e9; int bi = -1, bt = -1;
    for (int i = 0; i < nd; i++) {
      if (assigned[i]) continue;
      for (int k = 0; k < MAX_TRACKS; k++) {
        if (!tracks[k].active || taken[k]) continue;
        float db = fabsf(tracks[k].worldBrg - dets[i].worldBrg);
        float dr = fabsf(tracks[k].rangeMm - dets[i].rangeMm);
        float gateR = (dets[i].estimated || tracks[k].rangeEstimated) ? TRACK_GATE_MM * 3 : TRACK_GATE_MM;
        if (db > TRACK_GATE_DEG || dr > gateR) continue;
        float c = db / TRACK_GATE_DEG + dr / gateR;
        if (c < bestCost) { bestCost = c; bi = i; bt = k; }
      }
    }
    if (bi < 0) break;
    assigned[bi] = true; taken[bt] = true;
    Track &t = tracks[bt]; Det &d = dets[bi];
    if (d.radar) {
      t.lastRadar = d.tRadar; t.radarWorldBrg = d.radarWorld; t.speed = d.speed;
      t.rangeMm = t.rangeMm * 0.4f + d.rangeMm * 0.6f;
      t.rangeEstimated = false;
    } else if (now - t.lastRadar > 400) {
      t.rangeMm = t.rangeMm * 0.7f + d.rangeMm * 0.3f;
      t.rangeEstimated = true;
    }
    if (d.cam) { t.lastCam = d.tCam; t.camWorldBrg = d.camWorld; t.box = d.box; }
    t.worldBrg = t.worldBrg * 0.3f + d.worldBrg * 0.7f;
  }

  // Unassigned detections start new tracks (best-evidence first)
  for (int pri = 3; pri >= 1; pri--) {
    for (int i = 0; i < nd; i++) {
      if (assigned[i]) continue;
      int p = (dets[i].radar ? 1 : 0) + (dets[i].cam ? 2 : 0);
      if (p != pri) continue;
      int slot = -1;
      for (int k = 0; k < MAX_TRACKS; k++) if (!tracks[k].active) { slot = k; break; }
      if (slot < 0) break;
      Track &t = tracks[slot];
      t = Track();
      t.active = true; t.id = nextId++; if (nextId == 0) nextId = 1;
      t.born = now;
      t.worldBrg = dets[i].worldBrg; t.rangeMm = dets[i].rangeMm;
      t.rangeEstimated = dets[i].estimated; t.speed = dets[i].speed;
      if (dets[i].radar) { t.lastRadar = dets[i].tRadar; t.radarWorldBrg = dets[i].radarWorld; }
      if (dets[i].cam) { t.lastCam = dets[i].tCam; t.camWorldBrg = dets[i].camWorld; t.box = dets[i].box; }
      assigned[i] = true;
    }
  }

  // Expire
  for (auto &t : tracks) {
    if (!t.active) continue;
    uint32_t last = max(t.lastRadar, t.lastCam);
    if (now - last > TRACK_TIMEOUT_MS) t.active = false;
  }
}

static TrackSrc trackSrc(const Track &t, uint32_t now) {
  bool r = t.lastRadar && now - t.lastRadar < 400;
  bool c = t.lastCam && now - t.lastCam < VISION_STALE_MS;
  return (TrackSrc)((r ? SRC_RADAR : 0) | (c ? SRC_CAM : 0));
}

static float trackConf(const Track &t, uint32_t now) {
  return clampf(0.55f * freshness(now, t.lastRadar, 500) + 0.45f * freshness(now, t.lastCam, 500), 0, 1);
}

// ---------------------------------------------------------------------------
//  Pan control
// ---------------------------------------------------------------------------
static int priorityOf(TrackSrc s) {
  if (s == SRC_BOTH) return 3;
  if (s == SRC_CAM) return 2;
  if (s == SRC_RADAR) return FOLLOW_RADAR_ONLY ? 1 : 0;
  return 0;
}

static void control(uint32_t now, float dt) {
  float desired = panDeg;

  if (mode == MODE_MANUAL) {
    desired = manualPan;
  } else if (mode == MODE_AUTO) {
    // Keep the current lock unless something with stronger evidence appears
    Track *lock = nullptr; int lockPri = 0;
    for (auto &t : tracks) if (t.active && t.id == lockedId) { lock = &t; lockPri = priorityOf(trackSrc(t, now)); }
    Track *best = nullptr; int bestPri = 0; float bestR = 1e9;
    for (auto &t : tracks) {
      if (!t.active) continue;
      int p = priorityOf(trackSrc(t, now));
      if (p > bestPri || (p == bestPri && p > 0 && t.rangeMm < bestR)) { best = &t; bestPri = p; bestR = t.rangeMm; }
    }
    Track *target = (lock && lockPri > 0 && lockPri >= bestPri) ? lock : best;
    if (target && priorityOf(trackSrc(*target, now)) > 0) {
      lockedId = target->id;
      lastTargetMs = now;
      bool camFresh = now - target->lastCam < VISION_STALE_MS;
      desired = camFresh ? target->camWorldBrg : target->radarWorldBrg;
    } else {
      lockedId = -1;
      if (sweepOn && now - lastTargetMs > SWEEP_START_MS) {
        // Search: sweep the full pan range back and forth
        if (!sweeping) { sweeping = true; sweepDir = panDeg < 0 ? 1 : -1; }  // head for the far side
        float p = panDeg + sweepDir * SWEEP_DEG_S * dt;
        if (p >= PAN_MAX_DEG) { p = PAN_MAX_DEG; sweepDir = -1; }
        if (p <= PAN_MIN_DEG) { p = PAN_MIN_DEG; sweepDir = 1; }
        panDeg = p;
        writeServo(panDeg);
        return;
      }
      if (!sweepOn && IDLE_RETURN_MS && now - lastTargetMs > IDLE_RETURN_MS) desired = 0;
    }
    sweeping = false;
  } else {
    sweeping = false;
  }

  desired = clampf(desired, PAN_MIN_DEG, PAN_MAX_DEG);
  float err = desired - panDeg;
  if (fabsf(err) > PAN_DEADBAND_DEG || mode == MODE_MANUAL) {
    float step = err * PAN_GAIN;
    float maxStep = PAN_MAX_DEG_S * dt;
    panDeg = clampf(panDeg + clampf(step, -maxStep, maxStep), PAN_MIN_DEG, PAN_MAX_DEG);
    writeServo(panDeg);
  }
}

// ---------------------------------------------------------------------------
//  RC car follow: steer until the target is dead ahead (straight shot),
//  then drive straight at it and stop at CAR_STANDOFF_MM.
// ---------------------------------------------------------------------------
static void carTick(uint32_t now) {
#if CAR_ENABLED
  static uint32_t lastSend = 0;
  if (carStopPending) {
    carStopPending = false;
    for (int i = 0; i < 3; i++) carlink::send(carlink::CMD_STOP, 0, 0, 0);  // broadcast has no ACK; repeat
  }
  if (now - lastSend < CAR_SEND_MS) return;
  lastSend = now;

  if (!carFollow) { carState = CAR_OFF; carThrottle = carSteer = 0; return; }  // car's watchdog stops it

  Track *t = nullptr;
  for (auto &k : tracks) if (k.active && k.id == lockedId) t = &k;
  TrackSrc src = t ? trackSrc(*t, now) : SRC_NONE;
  bool usable = t && src != SRC_NONE && (src != SRC_RADAR || CAR_FOLLOW_RADAR_ONLY);

  int thr = 0, steer = 0;
  if (!usable) {
    carState = CAR_IDLE;
    carAligned = false;
  } else {
    // Servo base is fixed to the car, so world bearing == bearing from the car's nose
    bool camFresh = now - t->lastCam < VISION_STALE_MS;
    float brg = camFresh ? t->camWorldBrg : t->radarWorldBrg;
    float r = t->rangeMm;
    if (carAligned && fabsf(brg) > CAR_ALIGN_OUT_DEG) carAligned = false;
    if (!carAligned && fabsf(brg) < CAR_ALIGN_IN_DEG) carAligned = true;
    int dir = brg > 0 ? 1 : -1;  // + bearing = target to the right

    if (!carAligned) {
      if (r > CAR_STANDOFF_MM - CAR_STANDOFF_BAND_MM) {
        carState = CAR_TURN; thr = CAR_TURN_THROTTLE; steer = dir;      // arc forward toward it
      } else if (CAR_REVERSE_TURNS) {
        carState = CAR_BACKTURN; thr = -CAR_TURN_THROTTLE; steer = -dir; // reverse, nose swings toward it
      } else {
        carState = CAR_HOLD;
      }
    } else if (r > CAR_STANDOFF_MM + CAR_STANDOFF_BAND_MM) {
      float k = clampf((r - CAR_STANDOFF_MM) / (float)CAR_RAMP_MM, 0, 1);
      carState = CAR_APPROACH;
      thr = CAR_MIN_THROTTLE + (int)((CAR_MAX_THROTTLE - CAR_MIN_THROTTLE) * k);
    } else {
      carState = CAR_HOLD;  // lined up at standoff distance
    }
  }
  carThrottle = thr; carSteer = steer;
  carlink::send(carlink::CMD_DRIVE, thr, steer, CAR_WATCHDOG_MS);
#endif
}

// ---------------------------------------------------------------------------
//  Web
// ---------------------------------------------------------------------------
static void handleState() {
  uint32_t now = millis();
  String s;
  s.reserve(1600);
  char buf[400];
  bool radarOk = radar.lastFrameMs && now - radar.lastFrameMs < 1000;
  bool visionOk = visionAlive && now - visionLastOk < 1500;
  const char *m = mode == MODE_AUTO ? "auto" : mode == MODE_HOLD ? "hold" : "manual";
  snprintf(buf, sizeof(buf),
           "{\"t\":%lu,\"pan\":%.1f,\"mode\":\"%s\",\"lock\":%d,\"radarOk\":%s,\"visionOk\":%s,"
           "\"frame\":%lu,\"fw\":%u,\"fh\":%u,\"hfov\":%.1f,\"vfov\":%.1f,\"maxR\":%d,"
           "\"sweepOn\":%s,\"sweeping\":%s,\"panMin\":%d,\"panMax\":%d,\"car\":{\"on\":%s,\"st\":\"%s\",\"thr\":%d,\"steer\":%d,\"aligned\":%s,\"standoff\":%d},\"tracks\":[",
           (unsigned long)now, panDeg, m, lockedId, radarOk ? "true" : "false", visionOk ? "true" : "false",
           (unsigned long)jpegSeq, frameW, frameH, CAM_HFOV_DEG, CAM_VFOV_DEG, RADAR_MAX_RANGE_MM,
           sweepOn ? "true" : "false", sweeping ? "true" : "false", PAN_MIN_DEG, PAN_MAX_DEG,
           carFollow ? "true" : "false", CAR_STATE_NAME[carState], carThrottle, carSteer,
           carAligned ? "true" : "false", CAR_STANDOFF_MM);
  s += buf;
  bool first = true;
  for (auto &t : tracks) {
    if (!t.active) continue;
    TrackSrc src = trackSrc(t, now);
    if (src == SRC_NONE) continue;
    float brgS = t.worldBrg - panDeg;  // bearing relative to where the head points now
    float x = t.rangeMm * sinf(brgS * DEG_TO_RAD), y = t.rangeMm * cosf(brgS * DEG_TO_RAD);
    bool inFov = fabsf(brgS) < CAM_HFOV_DEG / 2;
    const char *st = src == SRC_BOTH ? "both" : src == SRC_CAM ? "cam" : "radar";
    snprintf(buf, sizeof(buf),
             "%s{\"id\":%u,\"st\":\"%s\",\"x\":%.0f,\"y\":%.0f,\"r\":%.0f,\"brg\":%.1f,\"conf\":%.2f,"
             "\"inFov\":%s,\"est\":%s,\"spd\":%d",
             first ? "" : ",", t.id, st, x, y, t.rangeMm, brgS, trackConf(t, now),
             inFov ? "true" : "false", t.rangeEstimated ? "true" : "false", t.speed);
    s += buf;
    if (src & SRC_CAM) {
      snprintf(buf, sizeof(buf), ",\"box\":[%.3f,%.3f,%.3f,%.3f]", t.box.cx, t.box.cy, t.box.w, t.box.h);
      s += buf;
    }
    s += "}";
    first = false;
  }
  s += "],\"raw\":[";
  first = true;
  if (radarOk) {
    for (auto &t : radar.targets) {
      if (!t.valid) continue;
      snprintf(buf, sizeof(buf), "%s[%d,%d]", first ? "" : ",", RADAR_X_SIGN * t.x_mm, t.y_mm);
      s += buf; first = false;
    }
  }
  s += "]}";
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", s);
}

static void handleFrame() {
  size_t len = 0;
  xSemaphoreTake(jpegMutex, portMAX_DELAY);
  if (jpegFrontLen) { memcpy(jpegSend, jpegFront, jpegFrontLen); len = jpegFrontLen; }
  xSemaphoreGive(jpegMutex);
  if (!len) { server.send(204); return; }
  server.sendHeader("Cache-Control", "no-store");
  server.send_P(200, "image/jpeg", (const char *)jpegSend, len);
}

static void handleCmd() {
  String m = server.arg("mode");
  if (m == "auto") mode = MODE_AUTO;
  else if (m == "hold") mode = MODE_HOLD;
  else if (m == "center") { mode = MODE_MANUAL; manualPan = 0; }
  else if (m == "manual") { mode = MODE_MANUAL; manualPan = clampf(server.arg("pan").toFloat(), PAN_MIN_DEG, PAN_MAX_DEG); }
  String sw = server.arg("sweep");
  if (sw == "on") sweepOn = true;
  else if (sw == "off") sweepOn = false;
  String c = server.arg("car");
  if (c == "on") carFollow = true;
  else if (c == "off" || c == "stop") { carFollow = false; carStopPending = true; }
  server.send(200, "text/plain", "ok");
}

static void startNetwork() {
  bool sta = false;
  if (strlen(WIFI_SSID)) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.printf("[net] joining %s", WIFI_SSID);
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) { delay(250); Serial.print('.'); }
    sta = WiFi.status() == WL_CONNECTED;
    Serial.println();
  }
  if (sta) {
    Serial.printf("[net] dashboard: http://%s/  or http://%s.local/\n", WiFi.localIP().toString().c_str(), MDNS_NAME);
    MDNS.begin(MDNS_NAME);
    if (WiFi.channel() != ESPNOW_CHANNEL)
      Serial.printf("[car] WARNING: router is on channel %d; set ESPNOW_CHANNEL %d in car_receiver.ino or use hotspot mode\n",
                    WiFi.channel(), WiFi.channel());
  } else {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS, ESPNOW_CHANNEL);
    apMode = true;
    Serial.printf("[net] hotspot \"%s\" (pw %s) -> http://%s/\n", AP_SSID, AP_PASS, WiFi.softAPIP().toString().c_str());
  }
#if CAR_ENABLED
  Serial.println(carlink::begin(apMode) ? "[car] ESP-NOW ready (follow is OFF until enabled on the dashboard)"
                                        : "[car] ESP-NOW init failed");
#endif
  server.on("/", [] { server.send_P(200, "text/html", DASHBOARD_HTML); });
  server.on("/state", handleState);
  server.on("/frame.jpg", handleFrame);
  server.on("/cmd", handleCmd);
  server.begin();
}

// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\nradar_vision_tracker starting");

  jpegFront = (uint8_t *)malloc(JPEG_MAX);
  jpegBack = (uint8_t *)malloc(JPEG_MAX);
  jpegSend = (uint8_t *)malloc(JPEG_MAX);
  if (!jpegFront || !jpegBack || !jpegSend) Serial.println("[mem] JPEG buffers failed to allocate");
  jpegMutex = xSemaphoreCreateMutex();

  ESP32PWM::allocateTimer(0);
  servo.setPeriodHertz(50);
  servo.attach(PIN_SERVO, SERVO_MIN_US, SERVO_MAX_US);
  writeServo(0);

  radar.begin(RADAR_SERIAL, PIN_RADAR_RX, PIN_RADAR_TX, RADAR_BAUD);
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  xTaskCreatePinnedToCore(visionTask, "vision", 12288, nullptr, 1, nullptr, 0);

  startNetwork();
}

void loop() {
  static uint32_t lastCtl = 0, lastLog = 0;
  radar.poll();
  server.handleClient();

  uint32_t now = millis();
  if (now - lastCtl >= 20) {
    float dt = (now - lastCtl) / 1000.0f;
    if (dt > 0.1f) dt = 0.02f;
    lastCtl = now;
    fuse(now);
    control(now, dt);
  }
  carTick(now);

  if (now - lastLog >= 1000) {
    lastLog = now;
    int n = 0;
    for (auto &t : tracks) if (t.active) n++;
    Serial.printf("[status] radar:%s vision:%s tracks:%d lock:%d pan:%.1f%s car:%s\n",
                  (radar.lastFrameMs && now - radar.lastFrameMs < 1000) ? "ok" : "--",
                  (visionAlive && now - visionLastOk < 1500) ? "ok" : "--", n, lockedId, panDeg, sweeping ? " (sweeping)" : "", CAR_STATE_NAME[carState]);
  }
}
