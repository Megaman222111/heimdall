#pragma once
#include <WiFi.h>
#include <esp_now.h>

// Packet format shared with car_receiver.ino (must match exactly).
namespace carlink {
const uint8_t MAGIC = 0xC4;
enum : uint8_t { CMD_MOVE = 0, CMD_STOP = 1, CMD_DRIVE = 2 };  // DRIVE = replace current command now

typedef struct __attribute__((packed)) {
  uint8_t magic;
  uint8_t type;
  int16_t throttle;   // -255..255
  int8_t steer;       // -1 left, 0 straight, 1 right
  uint16_t duration;  // ms (for DRIVE: watchdog timeout)
} Cmd;

static uint8_t peer[6] = CAR_MAC;
static bool ready = false;

inline bool begin(bool apMode) {
  if (esp_now_init() != ESP_OK) return false;
  esp_now_peer_info_t p = {};
  memcpy(p.peer_addr, peer, 6);
  p.channel = 0;  // current WiFi channel
  p.ifidx = apMode ? WIFI_IF_AP : WIFI_IF_STA;
  p.encrypt = false;
  ready = esp_now_add_peer(&p) == ESP_OK;
  return ready;
}

inline bool send(uint8_t type, int throttle, int steer, uint16_t duration) {
  if (!ready) return false;
  Cmd c{MAGIC, type, (int16_t)constrain(throttle, -255, 255), (int8_t)constrain(steer, -1, 1), duration};
  return esp_now_send(peer, (const uint8_t *)&c, sizeof(c)) == ESP_OK;
}
}  // namespace carlink
