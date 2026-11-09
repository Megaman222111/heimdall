"""Nerf gun, controlled by text commands to an Arduino over USB serial."""
import glob
import threading
import time

import serial

import config as C


def find_port():
    ports = glob.glob("/dev/serial/by-id/*rduino*") + sorted(glob.glob("/dev/ttyACM*"))
    return ports[0] if ports else None


class Gun(threading.Thread):
    """Keeps the Arduino connected. Sends nothing unless fire() is called."""

    def __init__(self):
        super().__init__(daemon=True, name="gun")
        self.ser = None
        self.want_fire = threading.Event()
        self._line = b""

    @property
    def ok(self):
        return self.ser is not None

    def fire(self):
        """Queue one shot. Returns immediately; False if not connected or already busy."""
        if self.ser is None or self.want_fire.is_set():
            return False
        self.want_fire.set()
        return True

    def cancel(self):
        self.want_fire.clear()

    def _shoot(self, ser):
        ser.write(C.GUN_COMMAND)
        ser.flush()
        self.want_fire.clear()

    def _read(self, ser):
        self._line += ser.read(64)
        while b"\n" in self._line:
            reply, self._line = self._line.split(b"\n", 1)
            if reply.strip():
                print(f"[gun] arduino: {reply.strip()[:80].decode(errors='replace')}", flush=True)

    def run(self):
        while True:
            port = find_port()
            if not port:
                time.sleep(2)
                continue
            try:
                with serial.Serial(port, C.GUN_BAUD, timeout=0.05, write_timeout=0.2) as ser:
                    time.sleep(C.GUN_RESET_WAIT_S)  # opening the port restarts the Uno
                    ser.reset_input_buffer()
                    self.ser = ser
                    print(f"[gun] Arduino ready on {port}", flush=True)
                    while True:
                        if self.want_fire.is_set():
                            self._shoot(ser)
                        self._read(ser)
            except Exception as e:  # includes termios.error when the Uno vanishes mid-open
                print(f"[gun] disconnected ({e}); retrying", flush=True)
            self.ser = None
            self.want_fire.clear()  # never fire a stale shot after reconnecting
            time.sleep(1)
