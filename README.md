# Heimdall: Radar + Vision Pan Tracker (Raspberry Pi 4)

**By LaPeace**

**Group Members:**
- Megh Mistry
- Eli Cuala
- Antonio Avendano
- Amay Kumar
- Spandan Gautam
- Daniel Phan
  
Heimdall is an automated radar sentry that watches a fence line day or night, at a fraction of the cost of commercial perimeter radar. This repository holds its tracking system.

A Raspberry Pi 4 reads an HLK-LD2450 radar and a Pi camera and detects people on its CPU. It fuses both sensors into up to 3 tracks and pans a servo to keep the chosen person centred. It also hosts a live dashboard over its own WiFi hotspot.

## Why Heimdall

On September 14, 2012, attackers disguised in U.S. uniforms breached Camp Bastion in Afghanistan, killing two Marines and destroying or damaging eight Harrier aircraft. That was not an isolated event: incidents at RAF bases, MacDill Air Force Base and Joint Base Andrews show that perimeter security still has persistent gaps.

Each of today's main approaches falls short:

| Approach | Limitation |
|---|---|
| Human sentries | Lose about 15% of their accuracy within 30 minutes |
| Cameras | Fail in darkness, fog and dust |
| Dedicated radar | $6,000 to $100,000+ per unit, on closed, proprietary platforms |

Heimdall pairs a low-cost mmWave radar with camera-based person detection, so the system keeps working when one sensor can't see. No Canadian base should be left exposed, and protecting one shouldn't take a six-figure system.

## System overview

### The turret

Heimdall is a multi-tier turret designed in Tinkercad. Its base, middle and top platforms are joined by support columns. An MG996R servo pans the upper assembly on a lazy Susan bearing. A solenoid paired with a flywheel cage forms the Nerf dart launcher.

### Sensors

- **HLK-LD2450 24 GHz mmWave radar:** tracks movement in any lighting or weather, including people the camera can't see.
- **Raspberry Pi camera:** feeds YOLO11n person detection, which runs on the Pi's CPU.

## Hardware

- Raspberry Pi 4
- HLK-LD2450 24 GHz mmWave radar
- Raspberry Pi camera (CSI ribbon)
- MG996R servo
- 4.8–6 V battery pack for the servo
- 1000 µF capacitor
- Turret frame: base, middle and top platforms, support columns, lazy Susan bearing
- Launcher: solenoid and flywheel cage

## Wiring

| From | To | Pi physical pin |
|---|---|---|
| LD2450 5V | 5V | 2 |
| LD2450 GND | GND | 9 |
| LD2450 TX | GPIO5 (UART3 RX) | 29 |
| LD2450 RX | GPIO4 (UART3 TX) | 7 |
| Servo orange (signal) | GPIO18 (hardware PWM) | 12 |
| Servo red | Battery pack + (4.8–6 V) | not connected to the Pi |
| Servo brown | Battery pack − | not connected directly to the Pi |
| Battery pack − | GND | 14 |
| Camera ribbon | CSI port labelled CAMERA | contacts face the HDMI ports |

> **Never power the servo from the Pi's 5V pins.** A 1000 µF capacitor across the battery pack (stripe on −) helps absorb current spikes. Leave slack in the ribbon and wires for ±90° of pan.

## Setup

Boot config (`/boot/firmware/config.txt`), then reboot:

```
dtparam=audio=off
dtoverlay=uart3
dtoverlay=pwm,pin=18,func=2
```

Python environment:

```bash
python3 -m venv --system-site-packages .venv
.venv/bin/pip install -r requirements.txt
```

## Running

```bash
sudo cp deploy/radar-tracker.service /etc/systemd/system/
sudo systemctl enable --now radar-tracker   # starts now and on every boot
sudo bash deploy/hotspot.sh                 # hotspot starts on every boot (add --now to start it immediately)
```

After that, plugging in the Pi is all it takes. Join the WiFi network `RadarTracker` (password `tracker123`) and open http://192.168.4.1/.

The Pi has no internet while the hotspot is up.

- To get back online until the next boot: `sudo nmcli connection down RadarTracker`
- To remove the hotspot for good: `sudo nmcli connection delete RadarTracker`

Logs: `journalctl -u radar-tracker -f`

**Dashboard buttons:** Auto (tracks people), Hold (freezes the servo), Center, Sweep, and a manual slider.

## How the tracking works

- **Matching:** a camera box and a radar target within 8° of each other are treated as the same person (green).
- **Camera only (yellow):** range is estimated from the box height, shown with `~`.
- **Radar only (cyan silhouette):** inside the camera's view it's marked "occluded?"; outside it, an edge arrow points toward it.
- **Servo priority:** radar + camera, then camera only, then radar only, with ties going to the closest person. It stays locked unless a better-confirmed person appears.
- **Search:** with nobody tracked, the head sweeps side to side.

## Calibration (`config.py`)

| Symptom | Fix |
|---|---|
| Head turns away from the person | `SERVO_INVERT = True` |
| Radar dots mirrored vs. the camera | `RADAR_X_SIGN = -1` |
| Camera boxes mirrored | `CAM_X_SIGN = -1` |
| Image upside down | `CAM_HFLIP = True` and `CAM_VFLIP = True` |
| Head isn't straight at centre | adjust `SERVO_CENTER_DEG` |
| Servo buzzes at the ends of travel | narrow `PAN_MIN_DEG` / `PAN_MAX_DEG` |
| Oscillates / hunts | lower `PAN_GAIN` or raise `PAN_DEADBAND_DEG` |
| Too sluggish | raise `PAN_GAIN` / `PAN_MAX_DEG_S` |
| Radar and camera never match | raise `MATCH_DEG`; check both face the same way |
