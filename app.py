#!/usr/bin/env python3
"""Radar + vision pan tracker for Raspberry Pi 4. Serves the dashboard over WiFi."""
import logging
import os
import threading

from flask import Flask, Response, jsonify, request, send_file

import config as C
from ld2450 import Radar
from servo import Servo
from tracker import Tracker
from vision import Vision

HERE = os.path.dirname(os.path.abspath(__file__))
os.chdir(HERE)

logging.getLogger("werkzeug").setLevel(logging.WARNING)
app = Flask(__name__)
tracker = Tracker(Servo())


@app.get("/")
def index():
    return send_file("dashboard.html")


@app.get("/state")
def state():
    r = jsonify(tracker.state())
    r.headers["Cache-Control"] = "no-store"
    return r


@app.get("/frame.jpg")
def frame():
    with tracker.lock:
        jpeg = tracker.jpeg
    if not jpeg:
        return Response(status=204)
    return Response(jpeg, mimetype="image/jpeg", headers={"Cache-Control": "no-store"})


@app.get("/cmd")
def cmd():
    tracker.command(request.args.get("mode"), request.args.get("pan"), request.args.get("sweep"))
    return "ok"


if __name__ == "__main__":
    Radar(C.RADAR_UART_ADDR, C.RADAR_BAUD, tracker.on_radar).start()
    Vision(tracker).start()
    threading.Thread(target=tracker.run_forever, daemon=True, name="control").start()
    host = os.environ.get("TRACKER_HOST", C.WEB_HOST)
    port = int(os.environ.get("TRACKER_PORT", C.WEB_PORT))
    print(f"[web] dashboard on http://{host}:{port}/", flush=True)
    app.run(host=host, port=port, threaded=True)
