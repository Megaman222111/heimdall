"""Fuses radar and camera detections into tracks and aims the pan servo."""
import math
import threading
import time
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import config as C

SRC_NONE, SRC_RADAR, SRC_CAM, SRC_BOTH = 0, 1, 2, 3
Box = Tuple[float, float, float, float, float]  # cx, cy, w, h, score (normalised)


def now_ms():
    return int(time.monotonic() * 1000)


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def freshness(now, t, tau):
    return math.exp(-(now - t) / tau) if t else 0.0


@dataclass
class Det:
    radar: bool = False
    cam: bool = False
    estimated: bool = False
    world: float = 0.0
    radar_world: float = 0.0
    cam_world: float = 0.0
    range_mm: float = 0.0
    speed: int = 0
    t_radar: int = 0
    t_cam: int = 0
    box: Optional[Box] = None


@dataclass
class Track:
    id: int
    world: float
    range_mm: float
    estimated: bool = False
    speed: int = 0
    last_radar: int = 0
    last_cam: int = 0
    radar_world: float = 0.0
    cam_world: float = 0.0
    box: Optional[Box] = None


@dataclass
class VisionFrame:
    dets: List[Box] = field(default_factory=list)
    pan: float = 0.0
    ms: int = 0


class Tracker:
    def __init__(self, servo):
        self.servo = servo
        self.lock = threading.RLock()
        self.pan = 0.0
        self.tracks: List[Track] = []
        self.next_id = 1
        self.locked_id = -1
        self.last_target_ms = 0
        self.mode = "auto"
        self.manual_pan = 0.0
        self.sweep_on = C.SWEEP_ENABLED
        self.sweeping = False
        self.sweep_dir = 1
        self.radar_targets = []
        self.radar_ms = 0
        self.vision = VisionFrame()
        self.vision_ok_ms = 0
        self.jpeg = b""
        self.jpeg_seq = 0
        self.servo.write(0.0)

    # ------------------------- sensor inputs -------------------------
    def on_radar(self, targets):
        with self.lock:
            self.radar_targets = targets
            self.radar_ms = now_ms()

    def head_snapshot(self):
        with self.lock:
            return self.pan, now_ms()

    def on_vision(self, dets, pan, stamp, jpeg):
        with self.lock:
            self.vision = VisionFrame(dets, pan, stamp)
            self.vision_ok_ms = now_ms()
            self.jpeg = jpeg
            self.jpeg_seq += 1

    # ------------------------- fusion -------------------------
    def _detections(self, now):
        dets = []
        if self.radar_ms and now - self.radar_ms < C.RADAR_STALE_MS:
            for tx, ty, speed in self.radar_targets:
                x = C.RADAR_X_SIGN * tx
                r = math.hypot(x, ty)
                if r > C.RADAR_MAX_RANGE_MM:
                    continue
                world = self.pan + math.degrees(math.atan2(x, ty))
                dets.append(Det(radar=True, world=world, radar_world=world, range_mm=r,
                                speed=speed, t_radar=self.radar_ms))
        n_radar = len(dets)

        vf = self.vision
        if vf.ms and now - vf.ms < C.VISION_STALE_MS:
            used = [False] * n_radar
            for box in vf.dets:
                world = vf.pan + C.CAM_X_SIGN * (box[0] - 0.5) * C.CAM_HFOV_DEG
                best, best_err = -1, C.MATCH_DEG
                for j in range(n_radar):
                    err = abs(dets[j].radar_world - world)
                    if not used[j] and err < best_err:
                        best, best_err = j, err
                if best >= 0:
                    used[best] = True
                    d = dets[best]
                else:
                    # No radar range: estimate from box height vs. a typical person
                    ang = math.radians(max(box[3], 1e-3) * C.CAM_VFOV_DEG)
                    rng = clamp(C.PERSON_HEIGHT_M * 1000 / (2 * math.tan(ang / 2)), 300, 8000)
                    d = Det(range_mm=rng, estimated=True)
                    dets.append(d)
                d.cam, d.cam_world, d.world, d.t_cam, d.box = True, world, world, vf.ms, box
        return dets

    def _associate(self, dets):
        assigned, taken = set(), set()
        for _ in range(len(dets)):
            best_cost, bi, bt = float("inf"), -1, -1
            for i, d in enumerate(dets):
                if i in assigned:
                    continue
                for k, t in enumerate(self.tracks):
                    if k in taken:
                        continue
                    db, dr = abs(t.world - d.world), abs(t.range_mm - d.range_mm)
                    gate_r = C.TRACK_GATE_MM * 3 if (d.estimated or t.estimated) else C.TRACK_GATE_MM
                    if db > C.TRACK_GATE_DEG or dr > gate_r:
                        continue
                    cost = db / C.TRACK_GATE_DEG + dr / gate_r
                    if cost < best_cost:
                        best_cost, bi, bt = cost, i, k
            if bi < 0:
                break
            assigned.add(bi)
            taken.add(bt)
            self._update_track(self.tracks[bt], dets[bi])
        return assigned

    def _update_track(self, t, d):
        now = now_ms()
        if d.radar:
            t.last_radar, t.radar_world, t.speed = d.t_radar, d.radar_world, d.speed
            t.range_mm = t.range_mm * 0.4 + d.range_mm * 0.6
            t.estimated = False
        elif now - t.last_radar > C.RADAR_STALE_MS:
            t.range_mm = t.range_mm * 0.7 + d.range_mm * 0.3
            t.estimated = True
        if d.cam:
            t.last_cam, t.cam_world, t.box = d.t_cam, d.cam_world, d.box
        t.world = t.world * 0.3 + d.world * 0.7

    def _spawn(self, dets, assigned):
        for pri in (3, 2, 1):  # best evidence first
            for i, d in enumerate(dets):
                if i in assigned or (d.radar + 2 * d.cam) != pri or len(self.tracks) >= C.MAX_TRACKS:
                    continue
                self.tracks.append(Track(
                    id=self.next_id, world=d.world, range_mm=d.range_mm, estimated=d.estimated,
                    speed=d.speed, last_radar=d.t_radar, last_cam=d.t_cam,
                    radar_world=d.radar_world, cam_world=d.cam_world, box=d.box))
                self.next_id += 1
                assigned.add(i)

    def fuse(self, now):
        dets = self._detections(now)
        assigned = self._associate(dets)
        self._spawn(dets, assigned)
        self.tracks = [t for t in self.tracks if now - max(t.last_radar, t.last_cam) <= C.TRACK_TIMEOUT_MS]

    @staticmethod
    def source(t, now):
        r = bool(t.last_radar) and now - t.last_radar < C.RADAR_STALE_MS
        c = bool(t.last_cam) and now - t.last_cam < C.VISION_STALE_MS
        return (SRC_RADAR if r else 0) | (SRC_CAM if c else 0)

    @staticmethod
    def priority(src):
        if src == SRC_BOTH:
            return 3
        if src == SRC_CAM:
            return 2
        if src == SRC_RADAR and C.FOLLOW_RADAR_ONLY:
            return 1
        return 0

    # ------------------------- pan control -------------------------
    def _pick_target(self, now):
        lock, lock_pri = None, 0
        best, best_pri, best_r = None, 0, float("inf")
        for t in self.tracks:
            p = self.priority(self.source(t, now))
            if t.id == self.locked_id:
                lock, lock_pri = t, p
            if p > best_pri or (p == best_pri and p > 0 and t.range_mm < best_r):
                best, best_pri, best_r = t, p, t.range_mm
        target = lock if (lock and lock_pri > 0 and lock_pri >= best_pri) else best
        return target if target and self.priority(self.source(target, now)) > 0 else None

    def control(self, now, dt):
        desired = self.pan
        if self.mode == "manual":
            desired = self.manual_pan
            self.sweeping = False
        elif self.mode == "auto":
            target = self._pick_target(now)
            if target:
                self.locked_id = target.id
                self.last_target_ms = now
                cam_fresh = now - target.last_cam < C.VISION_STALE_MS
                desired = target.cam_world if cam_fresh else target.radar_world
            else:
                self.locked_id = -1
                if self.sweep_on and now - self.last_target_ms > C.SWEEP_START_MS:
                    self._sweep(dt)
                    return
                if not self.sweep_on and C.IDLE_RETURN_MS and now - self.last_target_ms > C.IDLE_RETURN_MS:
                    desired = 0.0
            self.sweeping = False
        else:
            self.sweeping = False

        desired = clamp(desired, C.PAN_MIN_DEG, C.PAN_MAX_DEG)
        err = desired - self.pan
        if abs(err) > C.PAN_DEADBAND_DEG or self.mode == "manual":
            max_step = C.PAN_MAX_DEG_S * dt
            self.pan = clamp(self.pan + clamp(err * C.PAN_GAIN, -max_step, max_step),
                             C.PAN_MIN_DEG, C.PAN_MAX_DEG)
            self.servo.write(self.pan)

    def _sweep(self, dt):
        if not self.sweeping:
            self.sweeping = True
            self.sweep_dir = 1 if self.pan < 0 else -1  # head for the far side
        p = self.pan + self.sweep_dir * C.SWEEP_DEG_S * dt
        if p >= C.PAN_MAX_DEG:
            p, self.sweep_dir = C.PAN_MAX_DEG, -1
        if p <= C.PAN_MIN_DEG:
            p, self.sweep_dir = C.PAN_MIN_DEG, 1
        self.pan = p
        self.servo.write(self.pan)

    def run_forever(self):
        last = now_ms()
        next_log = last + 1000
        while True:
            time.sleep(0.02)
            now = now_ms()
            dt = (now - last) / 1000.0
            if dt > 0.1:
                dt = 0.02
            last = now
            with self.lock:
                self.fuse(now)
                self.control(now, dt)
                if now >= next_log:
                    next_log = now + 1000
                    s = self.state()
                    print(f"[status] radar:{'ok' if s['radarOk'] else '--'} "
                          f"vision:{'ok' if s['visionOk'] else '--'} tracks:{len(s['tracks'])} "
                          f"lock:{self.locked_id} pan:{self.pan:.1f}{' (sweeping)' if self.sweeping else ''}",
                          flush=True)

    # ------------------------- web API -------------------------
    def state(self):
        with self.lock:
            now = now_ms()
            radar_ok = bool(self.radar_ms) and now - self.radar_ms < 1000
            tracks = []
            for t in self.tracks:
                src = self.source(t, now)
                if src == SRC_NONE:
                    continue
                brg = t.world - self.pan  # relative to where the head points now
                rad = math.radians(brg)
                item = {
                    "id": t.id, "st": {SRC_BOTH: "both", SRC_CAM: "cam", SRC_RADAR: "radar"}[src],
                    "x": round(t.range_mm * math.sin(rad)), "y": round(t.range_mm * math.cos(rad)),
                    "r": round(t.range_mm), "brg": round(brg, 1),
                    "conf": round(clamp(0.55 * freshness(now, t.last_radar, 500)
                                        + 0.45 * freshness(now, t.last_cam, 500), 0, 1), 2),
                    "inFov": abs(brg) < C.CAM_HFOV_DEG / 2, "est": t.estimated, "spd": t.speed,
                }
                if (src & SRC_CAM) and t.box:
                    item["box"] = [round(v, 3) for v in t.box[:4]]
                tracks.append(item)
            return {
                "t": now, "pan": round(self.pan, 1), "mode": self.mode, "lock": self.locked_id,
                "radarOk": radar_ok, "visionOk": bool(self.vision_ok_ms) and now - self.vision_ok_ms < 1500,
                "servoOk": self.servo.ok, "frame": self.jpeg_seq, "fw": C.CAM_WIDTH, "fh": C.CAM_HEIGHT,
                "hfov": C.CAM_HFOV_DEG, "vfov": C.CAM_VFOV_DEG, "maxR": C.RADAR_MAX_RANGE_MM,
                "sweepOn": self.sweep_on, "sweeping": self.sweeping,
                "panMin": C.PAN_MIN_DEG, "panMax": C.PAN_MAX_DEG, "tracks": tracks,
                "raw": [[C.RADAR_X_SIGN * x, y] for x, y, _ in self.radar_targets] if radar_ok else [],
            }

    def command(self, mode, pan, sweep):
        with self.lock:
            if mode in ("auto", "hold"):
                self.mode = mode
            elif mode == "center":
                self.mode, self.manual_pan = "manual", 0.0
            elif mode == "manual":
                try:
                    self.manual_pan = clamp(float(pan), C.PAN_MIN_DEG, C.PAN_MAX_DEG)
                    self.mode = "manual"
                except (TypeError, ValueError):
                    pass
            if sweep in ("on", "off"):
                self.sweep_on = sweep == "on"
