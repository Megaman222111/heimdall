"""HLK-LD2450 radar reader.

Report frame (multi-target mode, 30 bytes, ~10 Hz):
    AA FF 03 00 | 3 x [x:2 y:2 speed:2 res:2] | 55 CC
Each int16 is sign-magnitude, with bit 15 set meaning positive.
"""
import glob
import os
import struct
import threading
import time

import serial

HEADER = b"\xaa\xff\x03\x00"
FOOTER = b"\x55\xcc"
FRAME_LEN = 30


def find_port(uart_addr):
    """Return the /dev/ttyAMA* device for a UART base address, or None."""
    for path in sorted(glob.glob("/sys/class/tty/ttyAMA*")):
        if uart_addr in os.path.realpath(os.path.join(path, "device")):
            return "/dev/" + os.path.basename(path)
    return None


def _sign_mag(v):
    return (v & 0x7FFF) if (v & 0x8000) else -(v & 0x7FFF)


def parse_frame(frame):
    """Return up to 3 targets as (x_mm, y_mm, speed_cms) from a complete frame."""
    targets = []
    for i in range(3):
        rx, ry, rs, _ = struct.unpack_from("<HHHH", frame, 4 + i * 8)
        if not (rx | ry | rs):
            continue
        x, y, speed = _sign_mag(rx), _sign_mag(ry), _sign_mag(rs)
        if y > 0:
            targets.append((x, y, speed))
    return targets


def extract_frames(buf):
    """Pull complete frames out of a bytearray, leaving any partial frame in it."""
    frames = []
    while True:
        start = buf.find(HEADER)
        if start < 0:
            del buf[: max(0, len(buf) - len(HEADER) + 1)]
            return frames
        del buf[:start]
        if len(buf) < FRAME_LEN:
            return frames
        if bytes(buf[FRAME_LEN - 2 : FRAME_LEN]) == FOOTER:
            frames.append(bytes(buf[:FRAME_LEN]))
            del buf[:FRAME_LEN]
        else:
            del buf[:1]  # false header; keep searching


class Radar(threading.Thread):
    def __init__(self, uart_addr, baud, on_targets):
        super().__init__(daemon=True, name="radar")
        self.uart_addr, self.baud, self.on_targets = uart_addr, baud, on_targets

    def run(self):
        while True:
            port = find_port(self.uart_addr)
            if not port:
                print(f"[radar] UART {self.uart_addr} not found (is dtoverlay=uart3 set?)", flush=True)
                time.sleep(5)
                continue
            try:
                with serial.Serial(port, self.baud, timeout=0.2) as s:
                    print(f"[radar] listening on {port}", flush=True)
                    buf = bytearray()
                    while True:
                        buf += s.read(max(1, s.in_waiting))
                        for frame in extract_frames(buf):
                            self.on_targets(parse_frame(frame))
            except (OSError, serial.SerialException) as e:
                print(f"[radar] {e}; retrying", flush=True)
                time.sleep(1)
