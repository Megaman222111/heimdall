#pragma once
#include <Arduino.h>

// HLK-LD2450 report frame (multi-target mode, default):
//   AA FF 03 00 | 3 x [x:2 y:2 speed:2 res:2] | 55 CC     (30 bytes, ~10 Hz)
// Each int16 field is sign-magnitude with bit 15 = 1 meaning positive.
struct RadarTarget {
  bool    valid;
  int16_t x_mm;      // + = right of sensor (before RADAR_X_SIGN)
  int16_t y_mm;      // distance in front of sensor
  int16_t speed_cms; // + = moving away
};

class LD2450 {
 public:
  RadarTarget targets[3];
  uint32_t lastFrameMs = 0;

  void begin(HardwareSerial &s, int rx, int tx, uint32_t baud) {
    _s = &s;
    _s->setRxBufferSize(1024);
    _s->begin(baud, SERIAL_8N1, rx, tx);
    memset(targets, 0, sizeof(targets));
  }

  // Returns true when a complete new frame was parsed.
  bool poll() {
    bool got = false;
    while (_s->available()) {
      uint8_t b = _s->read();
      if (_idx < 4) {                       // sync on header
        static const uint8_t H[4] = {0xAA, 0xFF, 0x03, 0x00};
        if (b == H[_idx]) _buf[_idx++] = b;
        else _idx = (b == 0xAA) ? (_buf[0] = b, 1) : 0;
        continue;
      }
      _buf[_idx++] = b;
      if (_idx == 30) {
        _idx = 0;
        if (_buf[28] == 0x55 && _buf[29] == 0xCC) { parse(); got = true; }
      }
    }
    return got;
  }

 private:
  HardwareSerial *_s = nullptr;
  uint8_t _buf[30];
  uint8_t _idx = 0;

  static int16_t sm(uint16_t v) { return (v & 0x8000) ? (int16_t)(v & 0x7FFF) : -(int16_t)(v & 0x7FFF); }

  void parse() {
    for (int i = 0; i < 3; i++) {
      const uint8_t *p = _buf + 4 + i * 8;
      uint16_t rx = p[0] | (p[1] << 8), ry = p[2] | (p[3] << 8), rs = p[4] | (p[5] << 8);
      RadarTarget &t = targets[i];
      t.valid = (rx | ry | rs) != 0;
      t.x_mm = sm(rx);
      t.y_mm = sm(ry);
      t.speed_cms = sm(rs);
      if (t.y_mm <= 0) t.valid = false;
    }
    lastFrameMs = millis();
  }
};
