// ESP-NOW RC car transmitter - ESP32 plugged into your computer
// Type timed commands into the Serial Monitor (115200 baud, line ending: Newline).
//
// Commands (speed 0-255 is optional, default 200; time in ms, max 10000):
//   F [speed] ms     forward              e.g.  F 1500   or  F 255 1500
//   B [speed] ms     backward             e.g.  B 150 1000
//   FL / FR [speed] ms   forward + steer left/right
//   BL / BR [speed] ms   backward + steer left/right
//   L ms / R ms      steer only, no drive
//   W ms             wait (car stopped)
//   S / STOP / X     stop now and clear anything queued (works mid-sequence)
//
// Chain commands with ';' to run them back to back:
//   F 1000; FL 800; W 500; B 150 1000; S
//
// Remote commands take priority over the tracker's follow mode: the car ignores
// follow commands while these run (+1 s after), and S/STOP pauses follow for 3 s.

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

const uint8_t MAGIC = 0xC4;
enum : uint8_t { CMD_MOVE = 0, CMD_STOP = 1 };

typedef struct __attribute__((packed)) {
  uint8_t  magic;
  uint8_t  type;
  int16_t  throttle;
  int8_t   steer;
  uint16_t duration;
} Cmd;

const int  DEFAULT_SPEED = 200;
const long MAX_DURATION  = 10000;
const int  MAX_CMDS      = 32;
const uint8_t ESPNOW_CHANNEL = 1;  // must match car_receiver.ino and the tracker

uint8_t broadcastAddr[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

void printHelp() {
  Serial.println();
  Serial.println("Commands (time in ms, speed 0-255 optional):");
  Serial.println("  F [speed] ms    B [speed] ms");
  Serial.println("  FL/FR/BL/BR [speed] ms");
  Serial.println("  L ms   R ms     W ms     S / STOP / X (stop now)");
  Serial.println("Chain with ';'  e.g.  F 1000; FL 800; W 500; B 150 1000");
  Serial.println();
}

// Parses one command like "FL 180 800". Returns false if invalid.
bool parseOne(String tok, Cmd &c) {
  tok.trim();
  tok.toUpperCase();
  if (tok.length() == 0) return false;

  c = {MAGIC, CMD_MOVE, 0, 0, 0};

  int sp = tok.indexOf(' ');
  String word = (sp < 0) ? tok : tok.substring(0, sp);
  String rest = (sp < 0) ? "" : tok.substring(sp + 1);
  rest.trim();

  long nums[2];
  int n = 0;
  while (rest.length() > 0 && n < 2) {
    int s = rest.indexOf(' ');
    String part = (s < 0) ? rest : rest.substring(0, s);
    nums[n++] = part.toInt();
    rest = (s < 0) ? "" : rest.substring(s + 1);
    rest.trim();
  }
  if (rest.length() > 0) return false;   // too many numbers

  if (word == "S" || word == "STOP" || word == "X") {
    if (n != 0) return false;
    c.type = CMD_STOP;
    return true;
  }

  long ms;
  if (word == "W") {
    if (n != 1) return false;
    ms = nums[0];
  } else if (word == "L" || word == "R") {
    if (n != 1) return false;
    c.steer = (word == "L") ? -1 : 1;
    ms = nums[0];
  } else {
    int dir = 0, st = 0;
    if      (word == "F")  { dir =  1; }
    else if (word == "B")  { dir = -1; }
    else if (word == "FL") { dir =  1; st = -1; }
    else if (word == "FR") { dir =  1; st =  1; }
    else if (word == "BL") { dir = -1; st = -1; }
    else if (word == "BR") { dir = -1; st =  1; }
    else return false;

    int speed = DEFAULT_SPEED;
    if (n == 1)      { ms = nums[0]; }
    else if (n == 2) { speed = nums[0]; ms = nums[1]; }
    else return false;

    c.throttle = dir * constrain(speed, 0, 255);
    c.steer = st;
  }

  if (ms <= 0) return false;
  c.duration = (uint16_t)constrain(ms, 1L, MAX_DURATION);
  return true;
}

void sendCmd(const Cmd &c) {
  if (esp_now_send(broadcastAddr, (const uint8_t *)&c, sizeof(Cmd)) != ESP_OK)
    Serial.println("  send failed");
  delay(5);
}

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (true) delay(1000);
  }

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, broadcastAddr, 6);
  peer.channel = 0;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("Failed to add broadcast peer");
    while (true) delay(1000);
  }

  Serial.println("Remote ready.");
  printHelp();
}

void loop() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;
  if (line.equalsIgnoreCase("help") || line == "?") { printHelp(); return; }

  // Parse the whole line first; send nothing if any part is invalid
  Cmd cmds[MAX_CMDS];
  int count = 0;
  int start = 0;
  while (true) {
    int semi = line.indexOf(';', start);
    String tok = (semi < 0) ? line.substring(start) : line.substring(start, semi);
    tok.trim();
    if (tok.length() > 0) {
      if (count >= MAX_CMDS) {
        Serial.println("Too many commands in one line (max 32)");
        return;
      }
      if (!parseOne(tok, cmds[count])) {
        Serial.print("Bad command: ");
        Serial.println(tok);
        return;
      }
      count++;
    }
    if (semi < 0) break;
    start = semi + 1;
  }

  for (int i = 0; i < count; i++) {
    const Cmd &c = cmds[i];
    if (c.type == CMD_STOP) Serial.println("-> STOP");
    else Serial.printf("-> throttle %d, steer %d, %u ms\n", c.throttle, c.steer, c.duration);
    sendCmd(c);
  }
}
