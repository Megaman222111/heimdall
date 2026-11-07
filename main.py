"""Real-time human body & movement detector.

Usage:
    python main.py                    # default webcam
    python main.py --source 1         # another camera
    python main.py --source clip.mp4  # a video file
    python main.py --follow           # keep you centered (dry run, no Arduino)
    python main.py --follow --port /dev/cu.usbmodem1101   # drive a real servo

Press q or Esc to quit.
"""
import argparse
from collections import defaultdict, deque

import cv2
import numpy as np
from ultralytics import YOLO

from follow import Follower, PanServo

HISTORY = 10      # frames of movement scores averaged per person (smooths jitter)
CONF_MIN = 0.5    # keypoints below this confidence are treated as unreliable

MOVING_COLOR = (0, 200, 0)
STILL_COLOR = (160, 160, 160)


def movement_score(prev, curr, conf, body_height):
    """How much one person moved between two consecutive frames.

    prev, curr:  (17, 2) arrays of keypoint (x, y) pixel positions (COCO order:
                 0 nose, 5-6 shoulders, 9-10 wrists, 11-12 hips, 15-16 ankles...)
    conf:        (17,) array of keypoint confidences for the current frame, 0-1
    body_height: height of the person's bounding box in pixels

    Returns a float: 0 means perfectly still, larger means more movement.
    """
    # TODO(you): implement — see notes in the chat.
    return 0.0  # placeholder: everyone shows as "still" until this is written


class MotionTracker:
    """Keeps per-person keypoint history and decides who is moving."""

    def __init__(self, still_threshold):
        self.still_threshold = still_threshold
        self.last_kpts = {}
        self.scores = defaultdict(lambda: deque(maxlen=HISTORY))

    def update(self, track_id, kpts, conf, body_height):
        """Feed one person's keypoints for this frame; returns (avg_score, is_moving)."""
        if track_id in self.last_kpts:
            score = movement_score(self.last_kpts[track_id], kpts, conf, body_height)
            self.scores[track_id].append(score)
        self.last_kpts[track_id] = kpts
        avg = float(np.mean(self.scores[track_id])) if self.scores[track_id] else 0.0
        return avg, avg > self.still_threshold

    def forget_missing(self, visible_ids):
        """Drop state for people who left the frame so memory doesn't grow forever."""
        for tid in list(self.last_kpts):
            if tid not in visible_ids:
                self.last_kpts.pop(tid, None)
                self.scores.pop(tid, None)


def process_frame(model, frame, tracker, follower=None):
    """Detect + track people in one frame and return the annotated image."""
    result = model.track(frame, persist=True, verbose=False)[0]
    canvas = result.plot(boxes=False, labels=False)  # draws skeletons only

    ids = [] if result.boxes.id is None else result.boxes.id.int().tolist()
    boxes = kpts = confs = None
    moving_count = 0
    if ids:
        boxes = result.boxes.xyxy.cpu().numpy()
        kpts = result.keypoints.xy.cpu().numpy()
        confs = result.keypoints.conf.cpu().numpy()
        for tid, box, kp, cf in zip(ids, boxes, kpts, confs):
            x1, y1, x2, y2 = box.astype(int)
            score, moving = tracker.update(tid, kp, cf, max(y2 - y1, 1))
            moving_count += moving
            color = MOVING_COLOR if moving else STILL_COLOR
            label = f"#{tid} {'MOVING' if moving else 'still'} {score:.3f}"
            cv2.rectangle(canvas, (x1, y1), (x2, y2), color, 2)
            cv2.putText(canvas, label, (x1, max(y1 - 8, 16)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    tracker.forget_missing(set(ids))
    if follower:
        follower.update(canvas, ids, boxes, kpts, confs)

    cv2.putText(canvas, f"People: {len(ids)}  Moving: {moving_count}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    return canvas


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", default="0", help="camera index or video file path")
    ap.add_argument("--model", default="yolo11n-pose.pt", help="any Ultralytics pose model")
    ap.add_argument("--still", type=float, default=0.01,
                    help="average movement score above which a person counts as moving")
    ap.add_argument("--follow", action="store_true", help="keep one person centered with a pan servo")
    ap.add_argument("--port", help="Arduino serial port, e.g. /dev/cu.usbmodem1101 (omit for dry run)")
    ap.add_argument("--invert", action="store_true", help="flip servo direction if it turns the wrong way")
    args = ap.parse_args()

    is_camera = args.source.isdigit()
    source = int(args.source) if is_camera else args.source
    cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        raise SystemExit(f"Could not open source {args.source!r}"
                         + (" (check System Settings → Privacy → Camera)" if is_camera else ""))

    model = YOLO(args.model)
    tracker = MotionTracker(args.still)
    servo = PanServo(args.port, invert=args.invert) if args.follow else None
    follower = Follower(servo, CONF_MIN) if servo else None

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            if is_camera:
                frame = cv2.flip(frame, 1)  # mirror so it feels like a mirror
            cv2.imshow("Body & Movement Detector", process_frame(model, frame, tracker, follower))
            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()
        if servo:
            servo.close()


if __name__ == "__main__":
    main()
