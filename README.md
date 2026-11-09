# Radar + Vision Pan Tracker (Raspberry Pi 4)

A Raspberry Pi 4 reads an HLK-LD2450 radar and a Pi camera, detects people on its
CPU, fuses both into up to 3 tracks, and pans a servo to keep the chosen person
centred. It hosts a live dashboard over its own WiFi hotspot.

Radar targets the camera can't see (behind a wall, or outside its field of view)
are drawn as glowing silhouettes.

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

Never power the servo from the Pi's 5V pins. A 1000 µF capacitor across the
battery pack (stripe on −) helps absorb current spikes. Leave slack in the
ribbon and wires for ±90° of pan.

## Setup

Boot config (`/boot/firmware/config.txt`), then reboot:

```
dtparam=audio=off
dtoverlay=uart3
dtoverlay=pwm,pin=18,func=2
```

Python environment:

```
python3 -m venv --system-site-packages .venv
.venv/bin/pip install -r requirements.txt
```

## Running

```
sudo cp deploy/radar-tracker.service /etc/systemd/system/
sudo systemctl enable --now radar-tracker   # starts now and on every boot
sudo bash deploy/hotspot.sh                 # hotspot starts on every boot (add --now to start it immediately)
```

After that, plugging in the Pi is all it takes. Join the WiFi network
**RadarTracker** (password `tracker123`) and open **http://192.168.4.1/**.

The Pi has no internet while the hotspot is up. To get back online until the
next boot: `sudo nmcli connection down RadarTracker`. To remove the hotspot for
good: `sudo nmcli connection delete RadarTracker`.

Logs: `journalctl -u radar-tracker -f`.

Dashboard buttons: **Auto** (tracks people), **Hold** (freezes the servo),
**Center**, **Sweep**, and a manual slider.

## Nerf gun

An Arduino Uno on USB receives `fire` plus a newline at 115200 baud (`GUN_BAUD`,
`GUN_COMMAND`). With `AUTO_FIRE_ON_START = True` the gun is live as soon as the
Pi powers on; use **Disarm** on the dashboard, or set it to `False`.

It fires only in Auto mode, at a camera-confirmed person within 3° of centre,
held across 2 camera frames with the head steady, at most once every 4 s. The
Uno takes about 3.4 s to spin up before each dart launches.
Radar-only targets never fire. Mount the barrel level at chest height or lower.

## How the tracking works

- **Matching:** a camera box and a radar target within 8° of each other are
  treated as the same person (green).
- **Camera only** (yellow): range is estimated from the box height, shown with `~`.
- **Radar only** (cyan silhouette): inside the camera's view it's marked
  "occluded?"; outside it, an edge arrow points toward it.
- **Servo priority:** radar + camera, then camera only, then radar only, with
  ties going to the closest person. It stays locked unless a better-confirmed
  person appears.
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

If the distance looks right but left/right barely changes as you walk across,
the radar is mounted 90° off and needs a quarter-turn.
