// Standalone person follower: Grove Vision AI V2 + XIAO ESP32S3 -> Nano pan servo.
// No laptop needed once flashed.
//
// The Grove Vision AI V2 runs a person-detection or pose model on its own camera
// and NPU. This XIAO reads the results over UART, works out how far to turn, and
// sends the angle to the Nano (running arduino/pan_servo) as text lines: "97\n".
// Same steering logic as follow.py on the laptop.
//
// It also hosts a live dashboard (camera view, detections, steering math, tuning):
//   - default: joins its own Wi-Fi "PersonFollower", open http://192.168.4.1
//   - or set WIFI_SSID/WIFI_PASS to join your network, open http://follower.local
//
// Wiring (XIAO -> Nano): D2 -> D0 (RX), GND -> GND. Data only flows one way, so
// the Nano's 5V TX never touches the 3.3V XIAO.

#include <Arduino.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WiFi.h>
#include <mbedtls/base64.h>
#include <Seeed_Arduino_SSCMA.h>

#include "dashboard.h"

const char *WIFI_SSID = "";  // leave empty to always use the ESP's own network
const char *WIFI_PASS = "";
const char *AP_SSID = "PersonFollower";
const char *AP_PASS = "follow123";  // at least 8 characters

const int NANO_TX_PIN = D2;  // D0/D1/D3-D10 are used by the Vision AI board's I2C/UART/SPI
const uint32_t NANO_BAUD = 115200;
const int VISION_RST_PIN = D3;

const int MIN_SCORE = 50;  // ignore detections/keypoints below 50% confidence
const int MIN_ANGLE = 0, MAX_ANGLE = 180;
const uint32_t RESEND_MS = 1000;   // repeat the angle so a Nano that rebooted catches up
const uint32_t VIEWER_MS = 3000;   // keep fetching camera images this long after a dashboard poll

struct Tuning {
  float gain = 5.0;       // degrees per frame at the very edge of the frame
  float deadZone = 0.08;  // |error| below this counts as centered
  float maxStep = 4.0;    // never turn more than this per frame
  bool invert = false;    // flip if the servo turns away from you
} tune;  // editable live from the dashboard, saved in flash

SSCMA AI;
HardwareSerial NanoSerial(1);
WebServer server(80);
Preferences prefs;

int frameW = 240, frameH = 240;  // replaced by the real size read from the camera's JPEG

// Latest result, served to the dashboard at /state.
uint32_t frameId = 0, lastResultMs = 0, lastPollMs = 0;
float fps = 0;
bool found = false;
float targetX = 0, error = 0, step = 0, angleBefore = 90;
String people = "[]";
String image;

float angle = 90;  // matches the Nano's starting position
int lastSent = -1;
uint32_t lastSentMs = 0;

float panStep(float error) {
  if (fabs(error) < tune.deadZone) return 0;  // close enough: hold still instead of twitching
  return constrain(tune.gain * error, -tune.maxStep, tune.maxStep);
}

void sendAngle() {
  int a = lround(angle);
  if (a == lastSent && millis() - lastSentMs < RESEND_MS) return;
  lastSent = a;
  lastSentMs = millis();
  NanoSerial.println(a);  // "97\r\n"; the Nano sketch ignores the '\r'
}

// Picks the biggest confident person and returns the horizontal pixel to center on:
// pose models -> average of the visible face keypoints (nose, eyes, ears), else box
// center; detection models -> box center. Also writes everyone found to `out` as
// JSON for the dashboard. Returns false if nobody is in view.
bool findTarget(float &x, String &out) {
  std::vector<keypoints_t> &kps = AI.keypoints();
  std::vector<boxes_t> &boxes = AI.boxes();
  bool pose = !kps.empty();
  size_t n = pose ? kps.size() : boxes.size();

  int best = -1;
  uint32_t bestArea = 0;
  for (size_t i = 0; i < n; i++) {
    boxes_t &b = pose ? kps[i].box : boxes[i];
    uint32_t area = (uint32_t)b.w * b.h;
    if (b.score >= MIN_SCORE && area > bestArea) { best = i; bestArea = area; }
  }

  char buf[64];
  out = "[";
  for (size_t i = 0; i < n; i++) {
    boxes_t &b = pose ? kps[i].box : boxes[i];
    snprintf(buf, sizeof buf, "%s{\"b\":[%u,%u,%u,%u,%u]", i ? "," : "", b.x, b.y, b.w, b.h, b.score);
    out += buf;
    if (pose) {
      out += ",\"k\":[";
      for (size_t j = 0; j < kps[i].points.size(); j++) {
        point_t &p = kps[i].points[j];
        snprintf(buf, sizeof buf, "%s[%u,%u,%u]", j ? "," : "", p.x, p.y, p.score);
        out += buf;
      }
      out += "]";
    }
    if ((int)i == best) out += ",\"t\":1";
    out += "}";
  }
  out += "]";

  if (best < 0) return false;
  if (!pose) {
    x = boxes[best].x;  // SSCMA boxes are center-based
    return true;
  }
  keypoints_t &kp = kps[best];
  float sum = 0;
  int seen = 0;
  for (size_t j = 0; j < 5 && j < kp.points.size(); j++) {
    if (kp.points[j].score >= MIN_SCORE) { sum += kp.points[j].x; seen++; }
  }
  x = seen ? sum / seen : kp.box.x;
  return true;
}

// Reads width/height from the JPEG's start-of-frame header. Detection coordinates
// are in the same pixel space as the image, so this keeps the error math honest.
void learnFrameSize(const String &b64) {
  static uint8_t jpg[3072];
  size_t len = min((size_t)b64.length(), (size_t)4096) & ~(size_t)3;
  size_t n = 0;
  if (mbedtls_base64_decode(jpg, sizeof jpg, &n, (const uint8_t *)b64.c_str(), len) != 0) return;
  for (size_t i = 2; i + 8 < n && jpg[i] == 0xFF;) {
    uint8_t marker = jpg[i + 1];
    if (marker >= 0xC0 && marker <= 0xC2) {
      int h = (jpg[i + 5] << 8) | jpg[i + 6], w = (jpg[i + 7] << 8) | jpg[i + 8];
      if (w > 0 && h > 0) { frameW = w; frameH = h; }
      return;
    }
    i += 2 + ((jpg[i + 2] << 8) | jpg[i + 3]);  // skip this segment
  }
}

void steer() {
  angleBefore = angle;
  found = findTarget(targetX, people);
  if (!found) { error = step = 0; return; }
  error = (targetX - frameW / 2.0f) / (frameW / 2.0f);
  step = panStep(error);
  angle = constrain(angle + (tune.invert ? -step : step), MIN_ANGLE, MAX_ANGLE);
  Serial.printf("x=%.0f/%d error=%+.2f step=%+.2f angle=%.1f\n", targetX, frameW, error, step, angle);
}

void handleState() {
  lastPollMs = millis();
  char head[420];
  snprintf(head, sizeof head,
           "{\"id\":%u,\"ms\":%u,\"fps\":%.1f,\"w\":%d,\"h\":%d,\"found\":%s,\"x\":%.1f,"
           "\"error\":%.4f,\"step\":%.3f,\"before\":%.2f,\"angle\":%.2f,\"sent\":%d,"
           "\"cfg\":{\"gain\":%.2f,\"dead\":%.3f,\"max\":%.2f,\"invert\":%s},\"people\":",
           frameId, lastResultMs, fps, frameW, frameH, found ? "true" : "false", targetX,
           error, step, angleBefore, angle, lround(angle),
           tune.gain, tune.deadZone, tune.maxStep, tune.invert ? "true" : "false");
  String json;
  json.reserve(strlen(head) + people.length() + image.length() + 16);
  json += head;
  json += people;
  json += ",\"image\":\"";
  json += image;
  json += "\"}";
  server.send(200, "application/json", json);
}

void handleSet() {
  if (server.hasArg("gain")) tune.gain = constrain(server.arg("gain").toFloat(), 0.1f, 30.0f);
  if (server.hasArg("dead")) tune.deadZone = constrain(server.arg("dead").toFloat(), 0.0f, 0.5f);
  if (server.hasArg("max")) tune.maxStep = constrain(server.arg("max").toFloat(), 0.1f, 30.0f);
  if (server.hasArg("invert")) tune.invert = server.arg("invert") == "1";
  prefs.putFloat("gain", tune.gain);
  prefs.putFloat("dead", tune.deadZone);
  prefs.putFloat("max", tune.maxStep);
  prefs.putBool("invert", tune.invert);
  server.send(200, "text/plain", "ok");
}

void startWifi() {
  if (strlen(WIFI_SSID)) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    for (int i = 0; i < 20 && WiFi.status() != WL_CONNECTED; i++) delay(500);
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("Dashboard: http://%s  or  http://follower.local\n", WiFi.localIP().toString().c_str());
  } else {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    Serial.printf("Join Wi-Fi \"%s\" (password %s), then open http://%s\n",
                  AP_SSID, AP_PASS, WiFi.softAPIP().toString().c_str());
  }
  WiFi.setSleep(false);  // power saving adds lag to every request
  MDNS.begin("follower");
}

void setup() {
  Serial.begin(115200);  // USB log for tuning; never wait for it, so this runs on battery
  NanoSerial.begin(NANO_BAUD, SERIAL_8N1, -1, NANO_TX_PIN);  // TX only

  prefs.begin("follow");
  tune.gain = prefs.getFloat("gain", tune.gain);
  tune.deadZone = prefs.getFloat("dead", tune.deadZone);
  tune.maxStep = prefs.getFloat("max", tune.maxStep);
  tune.invert = prefs.getBool("invert", tune.invert);

  startWifi();
  server.on("/", [] { server.send_P(200, "text/html", DASHBOARD_HTML); });
  server.on("/state", handleState);
  server.on("/set", handleSet);
  server.begin();

  // Serial0 = XIAO D6/D7, wired to the Vision AI board through the header.
  Serial0.setRxBufferSize(64 * 1024);  // camera frames arrive as ~10-30 KB of base64
  while (!AI.begin(&Serial0, VISION_RST_PIN)) {
    Serial.println("Grove Vision AI not responding, retrying...");
    server.handleClient();
    delay(1000);
  }
  Serial.println("Grove Vision AI connected");

  if (AI.invoke(1, false, true) == 0 && AI.last_image().length()) learnFrameSize(AI.last_image());
  Serial.printf("Frame size %dx%d\n", frameW, frameH);
  sendAngle();
}

void loop() {
  server.handleClient();

  bool viewer = lastPollMs && millis() - lastPollMs < VIEWER_MS;
  if (AI.invoke(1, false, viewer) == 0) {  // 0 = got a fresh result
    uint32_t now = millis();
    if (lastResultMs) fps = 0.8f * fps + 0.2f * 1000.0f / max(now - lastResultMs, (uint32_t)1);
    lastResultMs = now;
    image = viewer ? AI.last_image() : String();
    if (image.length()) learnFrameSize(image);
    steer();
    frameId++;
  }
  sendAngle();  // holds position (and keeps resending) while nobody is in view
}
