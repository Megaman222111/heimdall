"""Pi camera capture and on-CPU person detection (YOLO11n, ONNX Runtime)."""
import math
import threading
import time

import numpy as np
import onnxruntime as ort
import simplejpeg

import config as C

PERSON_CLASS = 0


class Detector:
    def __init__(self, path=C.MODEL_PATH):
        opts = ort.SessionOptions()
        opts.intra_op_num_threads = C.MODEL_THREADS
        self.session = ort.InferenceSession(path, opts, providers=["CPUExecutionProvider"])
        self.input_name = self.session.get_inputs()[0].name

    def detect(self, rgb):
        """Return person boxes as (cx, cy, w, h, score), normalised 0..1 to the frame."""
        h, w = rgb.shape[:2]
        step = max(1, math.ceil(max(w, h) / C.MODEL_SIZE))  # always fits, never crops
        small = rgb[::step, ::step]
        sh, sw = small.shape[:2]
        pad_x, pad_y = (C.MODEL_SIZE - sw) // 2, (C.MODEL_SIZE - sh) // 2
        canvas = np.full((C.MODEL_SIZE, C.MODEL_SIZE, 3), 127, dtype=np.uint8)
        canvas[pad_y : pad_y + sh, pad_x : pad_x + sw] = small
        blob = canvas.transpose(2, 0, 1)[None].astype(np.float32) / 255.0

        pred = self.session.run(None, {self.input_name: blob})[0][0].T  # N x (xywh + classes)
        scores = pred[:, 4 + PERSON_CLASS]
        pred, scores = pred[scores >= C.VISION_MIN_SCORE], scores[scores >= C.VISION_MIN_SCORE]
        out = []
        for i in _nms(pred[:, :4], scores, C.VISION_NMS_IOU):
            cx, cy, bw, bh = pred[i, :4]
            out.append((float((cx - pad_x) / sw), float((cy - pad_y) / sh),
                        float(bw / sw), float(bh / sh), float(scores[i])))
        return out


def _nms(xywh, scores, iou_limit):
    x1, y1 = xywh[:, 0] - xywh[:, 2] / 2, xywh[:, 1] - xywh[:, 3] / 2
    x2, y2 = xywh[:, 0] + xywh[:, 2] / 2, xywh[:, 1] + xywh[:, 3] / 2
    area = (x2 - x1) * (y2 - y1)
    order, keep = list(scores.argsort()[::-1]), []
    while order:
        i = order.pop(0)
        keep.append(i)
        rest = np.array(order, dtype=int)
        if not len(rest):
            break
        iw = np.clip(np.minimum(x2[i], x2[rest]) - np.maximum(x1[i], x1[rest]), 0, None)
        ih = np.clip(np.minimum(y2[i], y2[rest]) - np.maximum(y1[i], y1[rest]), 0, None)
        inter = iw * ih
        iou = inter / (area[i] + area[rest] - inter + 1e-9)
        order = list(rest[iou <= iou_limit])
    return keep


def _open_camera():
    from libcamera import Transform
    from picamera2 import Picamera2

    cam = Picamera2()
    # Choose the fastest sensor mode that still uses the whole sensor, so the
    # real field of view matches CAM_HFOV_DEG (OV5647's 640x480 mode is cropped).
    widest = max(m["crop_limit"][2] * m["crop_limit"][3] for m in cam.sensor_modes)
    full = [m for m in cam.sensor_modes if m["crop_limit"][2] * m["crop_limit"][3] == widest]
    mode = max(full, key=lambda m: m["fps"])
    cam.configure(cam.create_video_configuration(
        main={"size": (C.CAM_WIDTH, C.CAM_HEIGHT), "format": "RGB888"},
        raw={"size": mode["size"], "format": mode["unpacked"]},
        transform=Transform(hflip=C.CAM_HFLIP, vflip=C.CAM_VFLIP),
    ))
    cam.start()
    print(f"[vision] camera started, sensor mode {mode['size']}", flush=True)
    return cam


class Vision(threading.Thread):
    def __init__(self, tracker):
        super().__init__(daemon=True, name="vision")
        self.tracker = tracker
        self.fps = 0.0

    def run(self):
        detector = Detector()
        while True:
            try:
                cam = _open_camera()
            except Exception as e:
                print(f"[vision] camera unavailable ({e}); retrying", flush=True)
                time.sleep(5)
                continue
            self._loop(cam, detector)

    def _loop(self, cam, detector):
        frames, window = 0, time.monotonic()
        try:
            while True:
                pan, stamp = self.tracker.head_snapshot()
                bgr = cam.capture_array("main")  # picamera2's "RGB888" is B,G,R in memory
                dets = detector.detect(bgr[..., ::-1])
                jpeg = simplejpeg.encode_jpeg(bgr, quality=C.JPEG_QUALITY, colorspace="BGR")
                self.tracker.on_vision(dets, pan, stamp, jpeg)
                frames += 1
                if time.monotonic() - window >= 5:
                    self.fps = frames / (time.monotonic() - window)
                    frames, window = 0, time.monotonic()
                    print(f"[vision] {self.fps:.1f} detections/s", flush=True)
        except Exception as e:
            print(f"[vision] lost camera ({e}); reopening", flush=True)
            try:
                cam.close()
            except Exception:
                pass
            time.sleep(1)
