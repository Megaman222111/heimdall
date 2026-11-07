# body-detector

Detects people, tracks movement, and pans a servo to keep a person centered.
Two ways to run it:

| Mode | Vision runs on | Steering runs on | Servo driven by |
|---|---|---|---|
| Laptop (`main.py`) | Mac (YOLO11 pose) | Mac (`follow.py`) | Nano over USB |
| **Standalone** (`esp32/`) | Grove Vision AI V2 | XIAO ESP32S3 | Nano over a wire |

Both send the Nano the same thing — one angle per line (`"97\n"`) — so
`arduino/pan_servo` is identical in both modes.

## Standalone setup (no laptop)

### 1. Put a model on the Grove Vision AI V2 (once)
1. Plug the Grove Vision AI V2 **itself** into the Mac by its own USB-C.
2. In Chrome, open https://sensecraft.seeed.cc/ai → pretrained models.
3. Pick **Person Detection** (faster) or a **human pose** model (aims at the head).
4. Connect → Deploy. The model stays on the board after unplugging.

### 2. Flash the XIAO ESP32S3 (once)
Arduino IDE: Board **XIAO_ESP32S3**, library **Seeed Arduino SSCMA**,
open `esp32/grove_follow/grove_follow.ino`, Upload. The XIAO talks to the
Grove over its D6/D7 UART (through the header) with D3 as reset.

### 3. Flash the Nano (already done)
`arduino/pan_servo/pan_servo.ino`, Board **Arduino Nano**.
Disconnect the XIAO→D0 wire while uploading — it shares the Nano's RX pin.

### 4. Wiring
```
XIAO D2  ──────────► Nano D0 (RX)
XIAO GND ──────────► Nano GND
XIAO 5V  ──────────► Nano 5V        (powers the Nano)
Servo signal ──────► Nano D9
Servo red/brown ───► Nano 5V / GND  (SG90-size only; big servos need their own 5–6V)
Power bank ────────► XIAO USB-C     (powers XIAO + Grove Vision AI + Nano)
```
Leave the Nano's own USB **unplugged** when running standalone: its USB chip
also drives D0, and two supplies on 5V is asking for trouble.

### 5. Dashboard (phone or any browser)
The ESP32 makes its own Wi-Fi network — no router needed:
1. Join Wi-Fi **PersonFollower** (password `follow123`).
2. Open **http://192.168.4.1**.

To use your home network instead, set `WIFI_SSID` / `WIFI_PASS` at the top of
`grove_follow.ino`; then open **http://follower.local** (or the IP printed on the
XIAO's USB serial log). If it can't connect it falls back to its own network.

The dashboard shows what the Grove sees (with detections, the target, center
line and dead zone), the steering math for the current frame, the servo angle,
30-second charts of error and angle, and live **tuning** sliders that are saved
on the ESP32:
- Servo turns away from you → tick **Invert**.
- Laggy → raise **Gain**. Wobbles → lower **Gain** or widen **Dead zone**.

Camera frames are only pulled from the Grove while a dashboard is open, so
steering runs a bit faster when nobody is watching. The frame size is read from
the camera image automatically — no calibration needed.

## Laptop mode
```bash
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
.venv/bin/python main.py                 # detection only (model downloads on first run)
.venv/bin/python main.py --follow --port /dev/cu.usbserial-130   # drive the servo
```
`--source 1` picks another camera; omit `--port` for a dry run with no Arduino.
