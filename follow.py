"""Keep one person centered in frame by panning a servo through an Arduino.

Assumes the camera is mounted on the servo: turning the servo moves the view,
so we keep nudging it until the person's head sits in the middle of the frame.
"""
import time

import cv2
import numpy as np

DEAD_ZONE = 0.08  # |error| below this counts as "centered" (fraction of half-width)
GAIN = 5.0        # degrees per frame at the very edge of the frame; raise if laggy, lower if it wobbles
MAX_STEP = 4.0    # never turn more than this many degrees in one frame

FACE = [0, 1, 2, 3, 4]   # nose, eyes, ears
SHOULDERS = [5, 6]


def pan_step(error):
    """How many degrees to turn the servo this frame.

    error: where the target is horizontally, from -1.0 (left edge of frame)
           through 0.0 (dead center) to +1.0 (right edge).

    Returns degrees to add to the current servo angle (positive = turn right).
    Called once per frame, so ~20-30 times per second.
    """
    if abs(error) < DEAD_ZONE:
        return 0.0  # close enough to center: hold still instead of twitching
    return float(np.clip(GAIN * error, -MAX_STEP, MAX_STEP))


def aim_point(box, kpts, conf, conf_min):
    """Pixel (x, y) to center on: the head if visible, else shoulders, else upper box."""
    for group in (FACE, SHOULDERS):
        good = [i for i in group if conf[i] > conf_min]
        if good:
            x, y = kpts[good].mean(axis=0)
            return int(x), int(y)
    x1, y1, x2, y2 = box
    return int((x1 + x2) / 2), int(y1 + (y2 - y1) / 4)


class PanServo:
    """Sends servo angles to an Arduino over serial, or just tracks them (dry run)."""

    def __init__(self, port=None, baud=115200, invert=False, min_angle=0, max_angle=180):
        self.angle = 90.0
        self.invert = invert
        self.min_angle, self.max_angle = min_angle, max_angle
        self.serial = None
        self._last_sent = None
        if port:
            import serial
            self.serial = serial.Serial(port, baud, timeout=0)
            time.sleep(2)  # opening the port resets most Arduinos; wait for it to boot
        self._send()

    @property
    def dry_run(self):
        return self.serial is None

    def nudge(self, degrees):
        if self.invert:
            degrees = -degrees
        self.angle = float(np.clip(self.angle + degrees, self.min_angle, self.max_angle))
        self._send()

    def _send(self):
        angle = round(self.angle)
        if angle == self._last_sent:  # servos only take whole degrees; skip duplicates
            return
        self._last_sent = angle
        if self.serial:
            self.serial.write(f"{angle}\n".encode())

    def close(self):
        if self.serial:
            self.serial.close()


class Follower:
    """Picks one person to follow, steers the servo toward them, draws the guides."""

    def __init__(self, servo, conf_min):
        self.servo = servo
        self.conf_min = conf_min
        self.target_id = None

    def update(self, canvas, ids, boxes, kpts, confs):
        h, w = canvas.shape[:2]
        cx, band = w // 2, int(DEAD_ZONE * w / 2)
        cv2.line(canvas, (cx, 0), (cx, h), (255, 255, 255), 1)
        for x in (cx - band, cx + band):
            cv2.line(canvas, (x, 0), (x, h), (0, 200, 255), 1)

        status = "no one to follow"
        if ids:
            if self.target_id not in ids:  # lost them (or first run): lock onto the biggest person
                areas = (boxes[:, 2] - boxes[:, 0]) * (boxes[:, 3] - boxes[:, 1])
                self.target_id = ids[int(np.argmax(areas))]
            i = ids.index(self.target_id)
            x, y = aim_point(boxes[i], kpts[i], confs[i], self.conf_min)
            error = (x - cx) / (w / 2)
            self.servo.nudge(pan_step(error))

            centered = abs(error) < DEAD_ZONE
            color = (0, 200, 0) if centered else (0, 0, 255)
            cv2.circle(canvas, (x, y), 8, color, -1)
            cv2.line(canvas, (cx, y), (x, y), color, 2)
            status = f"following #{self.target_id}  error {error:+.2f}" + ("  CENTERED" if centered else "")

        mode = "DRY RUN" if self.servo.dry_run else "SERIAL"
        cv2.putText(canvas, f"{status}  |  servo {round(self.servo.angle)} deg ({mode})",
                    (10, h - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)
