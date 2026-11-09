#!/usr/bin/env python3
"""Radar + vision pan tracker for Raspberry Pi 4. Serves the dashboard over WiFi."""
import logging
import os
import threading

from flask import Flask, Response, jsonify, redirect, request, send_file

import config as C
from gun import Gun
from ld2450 import Radar
from servo import Servo
from tracker import Tracker
from vision import Vision

HERE = os.path.dirname(os.path.abspath(__file__))
os.chdir(HERE)

logging.getLogger("werkzeug").setLevel(logging.WARNING)
app = Flask(__name__)
gun = Gun()
tracker = Tracker(Servo(), gun)


@app.before_request
def log_visit():
    if request.path not in ("/state", "/frame.jpg"):
        print(f"[web] {request.remote_addr} {request.host}{request.path}", flush=True)


@app.errorhandler(404)
def captive_portal(_):
    # Phones probe a known URL after joining WiFi. Redirecting it marks this as a
    # sign-in network, so the phone opens the dashboard instead of avoiding the WiFi.
    return redirect("http://192.168.4.1/", code=302)


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
    args = request.args
    tracker.command(args.get("mode"), args.get("pan"), args.get("sweep"), args.get("arm"), args.get("fire"))
    return "ok"


if __name__ == "__main__":
    Radar(C.RADAR_UART_ADDR, C.RADAR_BAUD, tracker.on_radar).start()
    Vision(tracker).start()
    gun.start()
    threading.Thread(target=tracker.run_forever, daemon=True, name="control").start()
    host = os.environ.get("TRACKER_HOST", C.WEB_HOST)
    port = int(os.environ.get("TRACKER_PORT", C.WEB_PORT))
    print(f"[web] dashboard on http://{host}:{port}/", flush=True)
    app.run(host=host, port=port, threaded=True)
