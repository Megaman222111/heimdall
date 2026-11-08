// ESP-NOW RC car receiver - ESP32 + DRV8833
// Runs timed drive commands sent by remote_transmitter.ino, and live
// follow commands (CMD_DRIVE) sent by radar_vision_tracker.ino.
//
// Wiring (DRV8833 left column -> ESP32):
//   AIN1 -> D18   AIN2 -> D19   (drive motor)
//   BIN1 -> D21   BIN2 -> D22   (steering motor)
//   STBY -> D23   GND  -> GND
// Car battery goes to VM/GND on the driver. Power the ESP32 separately (USB / power bank).

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// ---------- Pins ----------
const int AIN1 = 18, AIN2 = 19;   // drive motor
const int BIN1 = 21, BIN2 = 22;   // steering motor
const int STBY = 23;              // driver enable

// ---------- Tuning ----------
const bool REVERSE_DRIVE = false;  // set true if "F" drives backward
const bool REVERSE_STEER = false;  // set true if "L" steers right
const int  PWM_FREQ   = 1000;      // Hz
const int  PWM_BITS   = 8;         // duty 0-255
const int  STEER_DUTY = 180;       // ~70%, limits stall current at the end stops
const int  RAMP_STEP  = 15;        // throttle change per ramp tick
const unsigned long RAMP_MS = 5;   // ramp tick interval
const uint16_t MAX_DURATION = 10000;  // ms, cap per command
const uint8_t ESPNOW_CHANNEL = 1;     // must match the tracker / transmitter
const unsigned long MANUAL_HOLDOFF_MS = 1000;  // ignore follow commands this long after a remote command
const unsigned long STOP_HOLDOFF_MS   = 3000;  // ...and this long after a STOP

// ---------- Protocol (must match transmitter) ----------
const uint8_t MAGIC = 0xC4;
enum : uint8_t { CMD_MOVE = 0, CMD_STOP = 1, CMD_DRIVE = 2 };
// CMD_DRIVE: replaces whatever is running immediately (no queueing).
// Its duration is a watchdog: the car stops if no new DRIVE arrives in time.

typedef struct __attribute__((packed)) {
  uint8_t  magic;
  uint8_t  type;      // CMD_MOVE or CMD_STOP
  int16_t  throttle;  // -255 (full reverse) .. 255 (full forward)
  int8_t   steer;     // -1 left, 0 straight, 1 right
  uint16_t duration;  // ms
} Cmd;

// ---------- State ----------
QueueHandle_t cmdQueue;
volatile bool stopRequested = false;
portMUX_TYPE liveMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool livePending = false;
unsigned long followBlockedUntil = 0;  // remote has priority over follow mode until this time
Cmd liveCmd;

Cmd current;
bool active = false;
unsigned long cmdStart = 0;
int targetThrottle = 0;
int currentThrottle = 0;
int steerDir = 0;
unsigned long lastRamp = 0;

// ---------- PWM helpers (Arduino-ESP32 core 2.x and 3.x) ----------
void pwmSetup(int pin, int ch) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(pin, PWM_FREQ, PWM_BITS);
#else
  ledcSetup(ch, PWM_FREQ, PWM_BITS);
  ledcAttachPin(pin, ch);
#endif
}

void pwmWrite(int pin, int ch, int duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, duty);
#else
  ledcWrite(ch, duty);
#endif
}

// Positive v drives in1, negative drives in2, 0 = coast.
// Never drives both inputs at once.
void setMotor(int in1, int ch1, int in2, int ch2, int v) {
  v = constrain(v, -255, 255);
  if (v > 0)      { pwmWrite(in2, ch2, 0); pwmWrite(in1, ch1, v); }
  else if (v < 0) { pwmWrite(in1, ch1, 0); pwmWrite(in2, ch2, -v); }
  else            { pwmWrite(in1, ch1, 0); pwmWrite(in2, ch2, 0); }
}

// ---------- ESP-NOW receive ----------
void handlePacket(const uint8_t *data, int len) {
  if (len != sizeof(Cmd)) return;
  Cmd c;
  memcpy(&c, data, sizeof(Cmd));
  if (c.magic != MAGIC) return;

  if (c.type == CMD_STOP) {   // stop preempts everything queued
    stopRequested = true;
    return;
  }
  if (c.type != CMD_MOVE && c.type != CMD_DRIVE) return;

  c.throttle = constrain(c.throttle, -255, 255);
  c.steer    = constrain(c.steer, -1, 1);
  if (c.duration > MAX_DURATION) c.duration = MAX_DURATION;

  if (c.type == CMD_DRIVE) {     // live command: hand straight to loop()
    portENTER_CRITICAL(&liveMux);
    liveCmd = c;
    livePending = true;
    portEXIT_CRITICAL(&liveMux);
    return;
  }
  xQueueSend(cmdQueue, &c, 0);   // dropped if queue is full
}

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  handlePacket(data, len);
}
#else
void onRecv(const uint8_t *mac, const uint8_t *data, int len) {
  handlePacket(data, len);
}
#endif

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);

  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, LOW);   // keep driver asleep during boot

  pwmSetup(AIN1, 0); pwmSetup(AIN2, 1);
  pwmSetup(BIN1, 2); pwmSetup(BIN2, 3);
  setMotor(AIN1, 0, AIN2, 1, 0);
  setMotor(BIN1, 2, BIN2, 3, 0);

  cmdQueue = xQueueCreate(32, sizeof(Cmd));

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (true) delay(1000);
  }
  esp_now_register_recv_cb(onRecv);

  digitalWrite(STBY, HIGH);  // wake the driver
  Serial.print("Car ready. MAC: ");
  Serial.println(WiFi.macAddress());
}

// ---------- Main loop ----------
void loop() {
  unsigned long now = millis();

  // Emergency stop: clear the queue and cut both motors immediately
  if (stopRequested) {
    stopRequested = false;
    followBlockedUntil = now + STOP_HOLDOFF_MS;
    livePending = false;
    xQueueReset(cmdQueue);
    active = false;
    targetThrottle = 0;
    currentThrottle = 0;
    steerDir = 0;
    Serial.println("STOP");
  }

  // Live follow command: replace everything immediately,
  // unless the remote is in control (its commands always win)
  bool remoteBusy = (active && current.type == CMD_MOVE) || uxQueueMessagesWaiting(cmdQueue) > 0 ||
                    (long)(now - followBlockedUntil) < 0;
  if (livePending && remoteBusy) livePending = false;   // drop it
  if (livePending) {
    Cmd c;
    portENTER_CRITICAL(&liveMux);
    c = liveCmd;
    livePending = false;
    portEXIT_CRITICAL(&liveMux);
    current = c;
    cmdStart = now;
    active = true;
    targetThrottle = c.throttle;
    steerDir = c.steer;
  }

  // Current command finished? (for DRIVE this is the watchdog timeout)
  if (active && now - cmdStart >= current.duration) {
    active = false;
    if (current.type == CMD_MOVE) followBlockedUntil = now + MANUAL_HOLDOFF_MS;
  }

  // Start the next queued command, or idle (failsafe: nothing queued = stopped)
  if (!active) {
    Cmd c;
    if (xQueueReceive(cmdQueue, &c, 0) == pdTRUE) {
      current = c;
      cmdStart = now;
      active = true;
      targetThrottle = c.throttle;
      steerDir = c.steer;
      Serial.printf("Run: throttle %d, steer %d, %u ms\n", c.throttle, c.steer, c.duration);
    } else {
      targetThrottle = 0;
      steerDir = 0;
    }
  }

  // Ramp throttle to limit current spikes and battery sag
  if (now - lastRamp >= RAMP_MS) {
    lastRamp = now;
    if (currentThrottle < targetThrottle)
      currentThrottle = min(currentThrottle + RAMP_STEP, targetThrottle);
    else if (currentThrottle > targetThrottle)
      currentThrottle = max(currentThrottle - RAMP_STEP, targetThrottle);
  }

  int drive = REVERSE_DRIVE ? -currentThrottle : currentThrottle;
  int steer = (REVERSE_STEER ? -steerDir : steerDir) * STEER_DUTY;
  setMotor(AIN1, 0, AIN2, 1, drive);
  setMotor(BIN1, 2, BIN2, 3, steer);
}
