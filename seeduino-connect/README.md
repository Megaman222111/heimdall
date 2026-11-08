# Radar + Vision Pan Tracker

An ESP32 reads the LD2450 radar and the Grove Vision AI V2's person detections.
It fuses them into up to 3 tracks and pans the sensor head with a servo to keep
the chosen person centred in frame. It also hosts a live dashboard over its own
WiFi hotspot. Everything runs from a battery, with no laptop.

Radar targets the camera can't see (behind a wall, or outside the camera's
field of view) are drawn as glowing "person" silhouettes:

- on the radar heatmap, sized by distance (closer = bigger);
- on the camera view, at the matching bearing, sized by perspective.

## Parts

| Have it | Part | Notes |
|---|---|---|
| ✅ | XIAO ESP32S3 (or an ESP32 DevKit) | **Replaces the Nano.** The Nano can't do it: the LD2450 needs a 256000-baud hardware UART, and the Seeed SSCMA library refuses to compile on AVR boards. |
| ✅ | HLK-LD2450 radar | |
| ✅ | Grove Vision AI V2 + OV5647 Pi camera | Must have a person-detection model flashed through SenseCraft AI |
| ✅ | Standard-size servo (MG996R-type) | |
| ✅ | Breadboard, jumpers | |
| ❌ | **USB power bank** with a 2–3 A port | Powers everything through the ESP32's USB |
| ❌ | **1000 µF 10 V+ electrolytic capacitor** | Across the servo supply. Stops servo current spikes from resetting the ESP32 |
| ❌ (maybe) | Grove-to-female-jumper cable | To plug the Grove V2's Grove port into the breadboard |
| optional | 2 × 4.7 kΩ resistors | I2C pull-ups to 3.3 V, only if vision connects intermittently |

No other microcontroller is needed. The Grove V2 runs the AI model itself, and
the ESP32 does radar, fusion, servo and WiFi.

## Wiring (XIAO ESP32S3, recommended)

The XIAO's **5V** pin outputs the USB 5 V, so it powers the LD2450 directly.
The servo has its own battery pack. No Nano is needed.

Connect the Grove V2 with its **Grove cable** (jumpers to the XIAO). Don't
stack the XIAO onto the Grove V2's socket: that socket also connects the
XIAO's D6/D7 to the Grove V2's own UART, which would clash with the radar.

| From | To XIAO ESP32S3 |
|---|---|
| LD2450 5V | **5V** |
| LD2450 GND | GND |
| LD2450 TX | **D7** |
| LD2450 RX | **D6** |
| Grove V2 red / black | **3V3** / GND |
| Grove V2 white (SDA) | **D4** |
| Grove V2 yellow (SCL) | **D5** |
| Servo orange (signal) | **D0** |
| Servo red / brown | servo battery pack + / – |
| Servo battery pack – | **XIAO GND** (shared ground, required) |

Power: USB power bank → XIAO USB-C. The XIAO only has one GND pin, so use the
breadboard – strip to join XIAO GND, LD2450 GND, Grove black and the servo
pack's –.

In Arduino, select board **XIAO_ESP32S3**. The pins switch automatically.

## Wiring (ESP32 DevKit, alternative)

| From | To ESP32 DevKit |
|---|---|
| LD2450 TX / RX | GPIO16 / GPIO17 |
| Grove V2 red / black / white / yellow | 3V3 / GND / GPIO21 / GPIO22 |
| Servo signal | GPIO18 |
| LD2450 5V | a 5 V source (e.g. the Nano's 5V pin, Nano powered by USB), all grounds shared |

Board: **ESP32 Dev Module**.

**Mechanical:** the plate with the radar and camera goes on the servo horn. The
Grove V2 board can ride on the plate or sit on the base. Leave enough slack in
the ribbon cable and wires for ±80° of rotation.

**Check the radar's orientation.** The LD2450 measures left/right along one
axis of the board. On the dashboard, walk across in front of it. If the
distance looks right but the left/right bearing barely changes, the module is
rotated 90° and needs remounting turned a quarter-turn. I couldn't confirm the
correct orientation from the photo.

## Flashing

1. Arduino IDE → Boards Manager → install **esp32 by Espressif**.
2. Library Manager → install **Seeed_Arduino_SSCMA** (it also installs ArduinoJson) and **ESP32Servo**.
3. Open `radar_vision_tracker/radar_vision_tracker.ino`, board **XIAO_ESP32S3** (or **ESP32 Dev Module**), upload.
4. Serial Monitor at 115200 prints `radar:ok vision:ok` once both sensors are talking.

I verified that it compiles cleanly for both boards on esp32 core 2.0.9.

## Using it

- With power on, the ESP32 creates the WiFi network **RadarTracker** (password `tracker123`).
- Join it from a phone and open **http://192.168.4.1/**.
- To use a router instead, set `WIFI_SSID` / `WIFI_PASS` in `config.h`. Then open
  `http://tracker.local/` or the IP shown in the serial log. If the router can't
  be reached, the ESP32 falls back to the hotspot.
- Buttons: **Auto** (tracks people), **Hold** (freezes the servo), **Center**, and a manual slider.

## How the tracking works

- **Matching:** both sensors are converted to a bearing relative to the servo
  base. A camera box and a radar target within 8° of each other are treated as
  the same person (green).
- **Camera only** (yellow): range is estimated from the box height and shown with `~`.
- **Radar only** (cyan silhouette): if the target is inside the camera's field of
  view, the camera should have seen it, so the dashboard marks it "occluded?"
  (likely behind a wall). If it's outside the field of view, an edge arrow
  points toward it.
- **Servo target priority:** radar + camera, then camera only, then radar only.
  Ties go to the closest person. The servo stays locked on its current person
  unless a better-confirmed one appears. With no camera detections it turns
  toward radar targets, so the camera can confirm them
  (`FOLLOW_RADAR_ONLY`).
- **Aiming:** the camera bearing is used for aiming when available because it's
  more precise; otherwise the radar bearing is used. A slew limit keeps the
  servo smooth.

## Calibration (in `config.h`)

| Symptom | Fix |
|---|---|
| Head turns away from the person | `SERVO_INVERT 1` |
| Radar dots are mirrored vs. the camera (a person on the right shows on the left in the heatmap) | `RADAR_X_SIGN -1` |
| Camera boxes mirrored | `CAM_X_SIGN -1` |
| Head isn't straight at 90° | adjust `SERVO_CENTER_DEG` |
| Oscillates / hunts | lower `PAN_GAIN` (e.g. 0.2) or raise `PAN_DEADBAND_DEG` |
| Too sluggish | raise `PAN_GAIN` / `PAN_MAX_DEG_S` |
| Radar and camera never match (always cyan + yellow for the same person) | raise `MATCH_DEG`; make sure the radar and camera face exactly the same way |
| Model uses a different class id for "person" | `VISION_PERSON_CLASS` |
| Vision drops out | add 4.7 kΩ pull-ups on SDA/SCL to 3.3 V |

Camera frames over I2C are slow (~0.5 s each), so the dashboard image updates
every 6th inference (`VISION_IMAGE_EVERY`) while detections run at full
speed. Vision runs on its own CPU core, so image transfers never stall the
servo.

## RC car follow mode

The tracker sends live drive commands over ESP-NOW to `car_receiver/car_receiver.ino`
(your receiver, with one addition). The sensor head + servo must be **mounted on
the car**, with servo centre pointing at the car's nose.

**What the car does** with the locked target:

1. **Turning:** the target is more than 12° off the nose, so the car arcs forward
   with the steering locked toward it. If it's already closer than the stop
   distance, it reverses with the opposite lock instead, which swings the nose
   toward the target.
2. **Straight shot:** once the target is within 6° of dead ahead, steering
   centres and the car drives straight at it. It goes faster when the target is
   far, and slows near the stop distance.
3. **Holding:** it stops 1.2 m short of the target (`CAR_STANDOFF_MM`) and stays lined up.

By default it only drives toward people the camera can see (green or yellow).
For radar-only (possibly behind-wall) targets, the servo turns to look but the
car holds still. Set `CAR_FOLLOW_RADAR_ONLY 1` to change that.

**Safety:**
- Follow mode is **off at boot**. Turn it on with the dashboard's *Follow* button.
- **STOP** sends an immediate stop to the car.
- Every drive command expires after 350 ms, so the car stops by itself if the
  tracker crashes, loses power or goes out of range.

**Setup**
1. Flash the updated `car_receiver.ino` to the car's ESP32. It still accepts
   your remote transmitter's commands, and now also live `CMD_DRIVE` commands.
2. Both sides use ESP-NOW channel 1, which is also the tracker hotspot's channel.
   Use hotspot mode while using the car. If the tracker joins a router on another
   channel, the serial log warns you; set the same `ESPNOW_CHANNEL` in
   `car_receiver.ino` to match.
3. Flash the updated `remote_transmitter/remote_transmitter.ino` too (it now locks to channel 1).
   The remote always wins over follow mode: the car ignores follow commands while a remote
   command runs and for 1 s after, and the remote's `S` / `STOP` pauses follow for 3 s.
4. Commands are broadcast by default. To target one car only, put the MAC the
   car prints at boot into `CAR_MAC` in `config.h`.

**Tuning (`config.h`)**

| Symptom | Fix |
|---|---|
| Car doesn't move / hums | raise `CAR_MIN_THROTTLE` / `CAR_TURN_THROTTLE` |
| Turns away from the target | `REVERSE_STEER = true` in car_receiver.ino |
| Drives backward when it should approach | `REVERSE_DRIVE = true` in car_receiver.ino |
| Zig-zags when lined up | widen `CAR_ALIGN_IN_DEG` / `CAR_ALIGN_OUT_DEG` |
| Overshoots the stop distance | lower `CAR_MAX_THROTTLE` or raise `CAR_STANDOFF_MM` |

**Limitation:** while the car is moving, the radar sees walls and furniture
shifting relative to it, which can create false targets. This is part of why
the car only drives toward camera-confirmed people by default.
