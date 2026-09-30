#!/usr/bin/env python3
"""
Raspberry Pi low-latency MJPEG streamer for a USB webcam (OpenCV + Flask).

Latency measures:
  * Asks the webcam for MJPG and, when possible, forwards the camera's own JPEG
    bytes untouched (no decode/re-encode on the Pi). Falls back to cv2.imencode
    automatically if the camera or OpenCV build can't do that.
  * CAP_PROP_BUFFERSIZE=1 plus a dedicated capture thread that reads
    continuously -> the driver never hands out an old queued frame.
  * One capture feeds every client; each client always gets the NEWEST frame
    (threading.Condition), so a slow client skips frames instead of queueing.

    sudo apt install -y python3-opencv python3-flask v4l-utils
    sudo iw dev wlan0 set power_save off
    v4l2-ctl --list-formats-ext -d /dev/video0     # see supported MJPG sizes/fps
    python3 pi_streamer.py --device 0 --width 640 --height 480 --fps 30

Stream URL: http://<pi-ip>:5000/video
"""

import argparse
import sys
import threading
import time

import cv2
import numpy as np
from flask import Flask, Response


class WebcamStream(threading.Thread):
    """Continuously grabs frames; keeps only the newest JPEG."""

    def __init__(self, device, width, height, fps, quality, passthrough):
        super().__init__(daemon=True)
        self.device, self.width, self.height, self.fps = device, width, height, fps
        self.quality, self.passthrough = quality, passthrough
        self.frame, self.seq = None, 0
        self.condition = threading.Condition()
        self.cap = None
        self.mode = "?"

    def _open(self):
        backend = cv2.CAP_V4L2 if sys.platform.startswith("linux") else cv2.CAP_ANY
        src = int(self.device) if str(self.device).isdigit() else self.device
        cap = cv2.VideoCapture(src, backend)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))   # set BEFORE size/fps
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if self.passthrough:
            cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)   # V4L2+MJPG: read() returns raw JPEG bytes
        return cap

    def _to_jpeg(self, frame):
        # Raw MJPEG pass-through comes back as a 1-row (or 1-D) uint8 buffer
        if self.passthrough and (frame.ndim == 1 or frame.shape[0] == 1):
            buf = frame.reshape(-1).tobytes()
            if buf[:2] == b"\xff\xd8":
                return buf
        if frame.ndim == 2 or frame.shape[0] == 1:
            return None
        ok, jpg = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.quality])
        return jpg.tobytes() if ok else None

    def _publish(self, jpg):
        with self.condition:
            self.frame = jpg
            self.seq += 1
            self.condition.notify_all()

    def run(self):
        while True:
            self.cap = self._open()
            if not self.cap.isOpened():
                print(f"Cannot open webcam {self.device}; retrying in 2 s")
                time.sleep(2)
                continue
            w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            f = self.cap.get(cv2.CAP_PROP_FPS)
            fourcc = int(self.cap.get(cv2.CAP_PROP_FOURCC)).to_bytes(4, "little").decode(errors="replace")
            print(f"Webcam opened: {w}x{h} @ {f:.0f} fps, format {fourcc}")

            checked = False
            fails = 0
            while True:
                ok, frame = self.cap.read()
                if not ok or frame is None:
                    fails += 1
                    if fails > 30:
                        print("Webcam read failing; reopening")
                        break
                    time.sleep(0.01)
                    continue
                fails = 0
                jpg = self._to_jpeg(frame)
                if not checked:
                    # verify pass-through JPEGs actually decode; otherwise switch to re-encode
                    raw = self.passthrough and (frame.ndim == 1 or frame.shape[0] == 1)
                    if raw and (jpg is None or cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR) is None):
                        print("MJPEG pass-through not usable; falling back to re-encoding")
                        self.passthrough = False
                        break
                    self.mode = "MJPEG pass-through (no re-encode)" if raw else f"OpenCV re-encode q={self.quality}"
                    print("Streaming mode:", self.mode)
                    checked = True
                if jpg is not None:
                    self._publish(jpg)
            self.cap.release()


def make_app(cam):
    app = Flask(__name__)

    def generate_frames():
        last_seq = -1
        while True:
            with cam.condition:
                if not cam.condition.wait_for(lambda: cam.seq != last_seq, timeout=2.0):
                    continue
                frame, last_seq = cam.frame, cam.seq
            yield (b"--frame\r\n"
                   b"Content-Type: image/jpeg\r\n"
                   b"Content-Length: " + str(len(frame)).encode() + b"\r\n\r\n"
                   + frame + b"\r\n")

    @app.route("/video")
    def video_feed():
        return Response(generate_frames(),
                        mimetype="multipart/x-mixed-replace; boundary=frame",
                        headers={"Cache-Control": "no-cache, no-store", "X-Accel-Buffering": "no"})

    @app.route("/snapshot.jpg")
    def snapshot():
        with cam.condition:
            cam.condition.wait_for(lambda: cam.frame is not None, timeout=2.0)
            frame = cam.frame
        return Response(frame, mimetype="image/jpeg")

    return app


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--device", default="0", help="webcam index (0 = /dev/video0) or device path")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--quality", type=int, default=75, help="JPEG quality when re-encoding")
    ap.add_argument("--no-passthrough", action="store_true", help="always decode + re-encode on the Pi")
    ap.add_argument("--port", type=int, default=5000)
    args = ap.parse_args()

    cam = WebcamStream(args.device, args.width, args.height, args.fps, args.quality,
                       passthrough=not args.no_passthrough)
    cam.start()
    print(f"Stream: http://<this-pi-ip>:{args.port}/video")
    make_app(cam).run(host="0.0.0.0", port=args.port, threaded=True)


if __name__ == "__main__":
    main()
