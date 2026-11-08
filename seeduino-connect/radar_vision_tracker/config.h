#pragma once
// ============================================================================
//  radar_vision_tracker — user configuration
//  Board: Seeed XIAO ESP32S3 ("XIAO_ESP32S3") or ESP32 DevKit ("ESP32 Dev Module")
// ============================================================================

// ---------------- Network ----------------
// Leave WIFI_SSID empty to always run as a standalone hotspot (no router needed).
// If set, the ESP32 tries to join that network for 10 s and falls back to the
// hotspot if it can't.
#define WIFI_SSID        ""
#define WIFI_PASS        ""
#define AP_SSID          "RadarTracker"
#define AP_PASS          "tracker123"     // min 8 chars
#define MDNS_NAME        "tracker"        // http://tracker.local (station mode)

// ---------------- Pins ----------------
// Chosen automatically from the board selected in Arduino:
//   "XIAO_ESP32S3"     -> XIAO pin labels below
//   "ESP32 Dev Module" -> DevKit GPIOs
#if defined(CONFIG_IDF_TARGET_ESP32S3)
  #define RADAR_SERIAL   Serial1
  #define PIN_RADAR_RX   D7   // XIAO D7 (GPIO44) <- LD2450 TX
  #define PIN_RADAR_TX   D6   // XIAO D6 (GPIO43) -> LD2450 RX
  #define PIN_I2C_SDA    D4   // XIAO D4 (GPIO5)  -> Grove V2 SDA (white)
  #define PIN_I2C_SCL    D5   // XIAO D5 (GPIO6)  -> Grove V2 SCL (yellow)
  #define PIN_SERVO      D0   // XIAO D0 (GPIO1)  -> servo signal
#else
  #define RADAR_SERIAL   Serial2
  #define PIN_RADAR_RX   16   // ESP32 RX2  <- LD2450 TX
  #define PIN_RADAR_TX   17   // ESP32 TX2  -> LD2450 RX
  #define PIN_I2C_SDA    21   // Grove V2 SDA (white)
  #define PIN_I2C_SCL    22   // Grove V2 SCL (yellow)
  #define PIN_SERVO      18   // servo signal
#endif

// ---------------- Sensors ----------------
#define RADAR_BAUD       256000
#define RADAR_X_SIGN     1     // set -1 if radar left/right is mirrored vs camera
#define RADAR_MAX_RANGE_MM 6000

#define CAM_HFOV_DEG     53.5f // OV5647 (Pi Camera v1.3) horizontal FOV
#define CAM_VFOV_DEG     41.4f
#define CAM_X_SIGN       1     // set -1 if camera image is mirrored
#define VISION_PERSON_CLASS 0  // class id of "person" in your SenseCraft model
#define VISION_MIN_SCORE 50    // 0-100
#define VISION_IMAGE_EVERY 6   // fetch a JPEG for the dashboard every N inferences
                               // (images over I2C are slow; 0 = never)
#define PERSON_HEIGHT_M  1.70f // used to estimate range for camera-only targets

// ---------------- Fusion / tracking ----------------
#define MAX_TRACKS       3
#define MATCH_DEG        8.0f  // radar<->vision bearing match gate
#define TRACK_GATE_DEG   12.0f // detection<->existing track gate
#define TRACK_GATE_MM    900
#define TRACK_TIMEOUT_MS 900
#define VISION_STALE_MS  450

// ---------------- Servo / pan control ----------------
#define SERVO_MIN_US     500
#define SERVO_MAX_US     2500
#define SERVO_CENTER_DEG 90    // servo angle where the head points straight ahead
#define PAN_MIN_DEG      -90   // limits relative to straight ahead (full 180° sweep)
#define PAN_MAX_DEG      90    // narrow these if the servo buzzes against its end stops
#define SERVO_INVERT     0     // 1 if a positive command turns the head the wrong way
#define PAN_GAIN         0.35f // fraction of error corrected per control tick (50 Hz)
#define PAN_MAX_DEG_S    60.0f // slew limit; also limits servo current spikes on USB power
#define PAN_DEADBAND_DEG 2.0f
#define FOLLOW_RADAR_ONLY 1    // turn toward radar-only targets when the camera sees nobody
#define IDLE_RETURN_MS   8000  // with sweep off: drift back to centre after this long (0 = hold)

// ---------------- Sweep (search when nobody is tracked) ----------------
#define SWEEP_ENABLED    1      // on at boot; toggle from the dashboard
#define SWEEP_START_MS   1500   // start sweeping this long after losing the target
#define SWEEP_DEG_S      25.0f  // sweep speed; slower = camera detects more reliably

// ---------------- RC car follow (ESP-NOW -> car_receiver.ino) ----------------
// Assumes the sensor head + servo are mounted on the car, servo centre = car forward.
#define CAR_ENABLED           1
#define CAR_MAC               {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} // broadcast, or the MAC car_receiver prints
#define ESPNOW_CHANNEL        1      // must match car_receiver.ino (hotspot runs on this channel)
#define CAR_SEND_MS           100    // command rate (10 Hz)
#define CAR_WATCHDOG_MS       350    // car stops by itself if it hears nothing for this long
#define CAR_STANDOFF_MM       1200   // stop this far from the person
#define CAR_STANDOFF_BAND_MM  250    // dead band around the standoff distance
#define CAR_ALIGN_IN_DEG      6.0f   // "straight shot": target within this of dead ahead
#define CAR_ALIGN_OUT_DEG     12.0f  // start steering again past this
#define CAR_MIN_THROTTLE      110    // below this most toy motors stall (0-255)
#define CAR_MAX_THROTTLE      170
#define CAR_TURN_THROTTLE     120    // speed while steering toward the target
#define CAR_RAMP_MM           2000   // full approach speed when this far beyond the standoff
#define CAR_FOLLOW_RADAR_ONLY 0      // 1 = also drive toward radar-only (possibly behind-wall) targets
#define CAR_REVERSE_TURNS     1      // back up while steering when too close to turn going forward
