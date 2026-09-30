#!/usr/bin/env python3
"""
Low-latency web GUI for a Raspberry Pi MJPEG stream + classic image processing.

    # OpenCV 4.x:  pip install fastapi "uvicorn[standard]" "opencv-python<5" numpy
    # OpenCV 5.x:  pip install fastapi "uvicorn[standard]" opencv-contrib-python numpy
    #   (5.0 moved CascadeClassifier to contrib and stopped shipping the .xml
    #    files; missing cascades are downloaded into ./cascades automatically)
    python laptop_app.py --url http://192.168.137.150:5000/video   # Raspberry Pi
    python laptop_app.py --esp32 192.168.1.50                      # ESP32-CAM sketch
    python laptop_app.py --cam 0                                   # webcam on this laptop
    # then open http://localhost:8000

    Any extra cascade .xml you put in ./cascades shows up in the detector dropdown.

Why this is fast (vs. the Gradio version):
  1. Grabber thread reads the Pi stream continuously and keeps ONLY the newest
     JPEG, so no stale frames pile up in socket buffers (the growing-delay bug).
  2. Processor thread runs every operation on that newest frame, then
     JPEG-encodes all panels in parallel (OpenCV releases the GIL).
  3. All panels go to the browser in ONE binary WebSocket message per frame.
     The browser decodes them natively (createImageBitmap) onto <canvas>.
  4. The browser acks each frame ("ready") before the server sends another, so
     a slow tab skips frames instead of buffering them.
"""

import argparse
import asyncio
import copy
import json
import os
import struct
import threading
import time
import urllib.request
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime

import cv2
import numpy as np
import uvicorn
from fastapi import FastAPI, WebSocket
from fastapi.responses import FileResponse

HERE = os.path.dirname(os.path.abspath(__file__))
SOI, EOI = b"\xff\xd8", b"\xff\xd9"

# ---------------------------------------------------------------------------
# Cascade (.xml) detectors -- works on OpenCV 4.x and on 5.x (contrib)
# ---------------------------------------------------------------------------
CASCADE_DIR = os.path.join(HERE, "cascades")
CASCADE_FILES = {   # dropdown name -> file name
    "face": "haarcascade_frontalface_default.xml",
    "face (alt2)": "haarcascade_frontalface_alt2.xml",
    "face (LBP)": "lbpcascade_frontalface_improved.xml",
    "eye": "haarcascade_eye.xml",
    "eye + glasses": "haarcascade_eye_tree_eyeglasses.xml",
    "profile": "haarcascade_profileface.xml",
    "smile": "haarcascade_smile.xml",
    "upperbody": "haarcascade_upperbody.xml",
    "fullbody": "haarcascade_fullbody.xml",
    "cat face": "haarcascade_frontalcatface.xml",
    "plate (RU)": "haarcascade_russian_plate_number.xml",
}
# user-supplied cascades: every *.xml in ./cascades not already listed
if os.path.isdir(CASCADE_DIR):
    for _f in sorted(os.listdir(CASCADE_DIR)):
        if _f.endswith(".xml") and _f not in CASCADE_FILES.values():
            CASCADE_FILES[os.path.splitext(_f)[0]] = _f

CASCADE_URLS = [  # tried in order when a file isn't found locally
    "https://raw.githubusercontent.com/opencv/opencv_contrib/5.x/modules/xobjdetect/data/{kind}/{file}",
    "https://raw.githubusercontent.com/opencv/opencv/4.x/data/{kind}/{file}",
]
_cascades, _cascade_lock = {}, threading.Lock()


def cascade_path(file):
    """./cascades -> OpenCV's bundled folder (4.x) -> download into ./cascades."""
    local = os.path.join(CASCADE_DIR, file)
    if os.path.isfile(local):
        return local
    bundled = os.path.join(getattr(getattr(cv2, "data", None), "haarcascades", "") or "", file)
    if os.path.isfile(bundled):
        return bundled
    kind = "lbpcascades" if file.startswith("lbp") else "haarcascades"
    os.makedirs(CASCADE_DIR, exist_ok=True)
    for url in CASCADE_URLS:
        try:
            with urllib.request.urlopen(url.format(kind=kind, file=file), timeout=15) as r:
                data = r.read()
            with open(local, "wb") as fh:
                fh.write(data)
            print(f"Downloaded cascade {file} ({len(data) // 1024} KB) -> {CASCADE_DIR}")
            return local
        except Exception:
            continue
    raise FileNotFoundError(f"cascade {file} not found locally and download failed; "
                            f"put it in {CASCADE_DIR}")


def get_cascade(name):
    with _cascade_lock:
        if name not in _cascades:
            c = cv2.CascadeClassifier(cascade_path(CASCADE_FILES[name]))
            if c.empty():
                raise ValueError(f"cascade '{name}' failed to load")
            _cascades[name] = c
        return _cascades[name]

# ---------------------------------------------------------------------------
# Panel + control definitions (single source of truth; the page builds itself
# from this). Every control key is also a key in DEFAULT_PARAMS.
# ---------------------------------------------------------------------------
def rng(key, label, lo, hi, step):
    return {"key": key, "type": "range", "label": label, "min": lo, "max": hi, "step": step}


def sel(key, label, options):
    return {"key": key, "type": "select", "label": label, "options": options}


MORPH_SHAPES = ["rect", "ellipse", "cross"]

PANELS = [
    {"id": "original",   "title": "Original (camera)",
     "controls": [sel("orientation", "input orientation", ["none", "rotate 180", "mirror", "flip vertical"])]},
    {"id": "resize",     "title": "1 · Resize",                "controls": [rng("proc_width", "width (px)", 160, 1280, 16)]},
    {"id": "color",      "title": "2 · Color conversion",      "controls": [sel("color_mode", "space", ["GRAY", "HSV", "LAB", "YCrCb", "RGB"])]},
    {"id": "haar",       "title": "3a · Cascade detection (Haar / LBP)",
     "controls": [sel("haar_model", "cascade", list(CASCADE_FILES)),
                  rng("haar_det_width", "detect width", 160, 640, 16),
                  rng("haar_scale", "scaleFactor", 1.05, 1.5, 0.05),
                  rng("haar_neighbors", "minNeighbors", 1, 10, 1)]},
    {"id": "otsu",       "title": "3b · Otsu segmentation",
     "controls": [sel("otsu_blur", "pre-blur", ["none", "gaussian5"]),
                  sel("otsu_invert", "invert", ["no", "yes"])]},
    {"id": "brightness", "title": "4 · Brightness",            "controls": [rng("brightness", "β (offset)", -128, 128, 1)]},
    {"id": "contrast",   "title": "5 · Contrast",              "controls": [rng("contrast", "α (gain)", 0.1, 3.0, 0.05)]},
    {"id": "huesat",     "title": "6 · Hue / Saturation",
     "controls": [rng("hue_shift", "hue shift (°/2)", -90, 90, 1),
                  rng("sat_scale", "saturation ×", 0.0, 3.0, 0.05)]},
    {"id": "clahe",      "title": "7 · CLAHE",
     "controls": [rng("clahe_clip", "clipLimit", 0.5, 10.0, 0.5),
                  rng("clahe_tiles", "tile grid", 2, 16, 1)]},
    {"id": "translate",  "title": "8a · Translation",
     "controls": [rng("tx", "tx (% width)", -50, 50, 1), rng("ty", "ty (% height)", -50, 50, 1)]},
    {"id": "rotate",     "title": "8b · Rotation",
     "controls": [rng("angle", "angle (°)", -180, 180, 1), rng("rot_scale", "scale", 0.2, 2.0, 0.05)]},
    {"id": "flip",       "title": "8c · Flip",                 "controls": [sel("flip_mode", "axis", ["horizontal", "vertical", "both"])]},
] + [
    {"id": op, "title": f"9{c} · {name} (on Otsu mask)",
     "controls": [rng(f"{op}_k", "kernel size", 1, 31, 2),
                  rng(f"{op}_iter", "iterations", 1, 10, 1),
                  sel(f"{op}_shape", "kernel shape", MORPH_SHAPES)]}
    for op, c, name in [("dilate", "a", "Dilation"), ("erode", "b", "Erosion"),
                        ("open", "c", "Opening"), ("close", "d", "Closing")]
] + [
    {"id": "denoise",    "title": "10 · Noise removal",
     "controls": [sel("noise_inject", "add noise first", ["none", "salt-pepper", "gaussian"]),
                  sel("denoise_method", "filter", ["median", "gaussian", "bilateral"]),
                  rng("denoise_k", "kernel size", 3, 15, 2)]},
    {"id": "retrieval",  "title": "11 · Image retrieval (HSV histogram)",
     "controls": [sel("retrieval_metric", "metric", ["correlation", "bhattacharyya", "intersection", "chi-square"])]},
]

DEFAULT_PARAMS = {
    "proc_width": 320, "jpeg_quality": 80, "orientation": "none",
    "color_mode": "HSV",
    "haar_model": "face", "haar_det_width": 320, "haar_scale": 1.2, "haar_neighbors": 5,
    "otsu_blur": "gaussian5", "otsu_invert": "no",
    "brightness": 50, "contrast": 1.5,
    "hue_shift": 30, "sat_scale": 1.5,
    "clahe_clip": 2.0, "clahe_tiles": 8,
    "tx": 15, "ty": 10, "angle": 30, "rot_scale": 1.0, "flip_mode": "horizontal",
    "noise_inject": "salt-pepper", "denoise_method": "median", "denoise_k": 5,
    "retrieval_metric": "correlation",
    "enabled": [p["id"] for p in PANELS],
}
for op in ("dilate", "erode", "open", "close"):
    DEFAULT_PARAMS.update({f"{op}_k": 5, f"{op}_iter": 1, f"{op}_shape": "rect"})

HAAR_FILES = CASCADE_FILES   # backward-compat alias


# ---------------------------------------------------------------------------
# 1. Grabber: keep only the newest JPEG from the MJPEG stream
# ---------------------------------------------------------------------------
class LatestFrameGrabber(threading.Thread):
    def __init__(self, url):
        super().__init__(daemon=True)
        self.url = url
        self.cond = threading.Condition()
        self.jpg, self.seq, self.t_recv = None, 0, 0.0
        self.skipped = 0           # frames that arrived but were superseded before use
        self.connected, self.error = False, ""

    def wait_new(self, last_seq, timeout=1.0):
        with self.cond:
            self.cond.wait_for(lambda: self.seq != last_seq, timeout=timeout)
            return self.jpg, self.seq, self.t_recv

    def _publish(self, jpg, skipped):
        with self.cond:
            self.jpg, self.t_recv = jpg, time.time()
            self.seq += 1
            self.skipped += skipped
            self.cond.notify_all()

    def run(self):
        while True:
            try:
                with urllib.request.urlopen(self.url, timeout=5) as resp:
                    self.connected, self.error = True, ""
                    buf = bytearray()
                    while True:
                        # read1 returns as soon as ANY data is available; read(n)
                        # would wait for n bytes and hold a finished frame hostage.
                        chunk = resp.read1(65536)
                        if not chunk:
                            break
                        buf += chunk
                        pos, newest, n = 0, None, 0
                        while True:
                            s = buf.find(SOI, pos)
                            if s < 0:
                                pos = max(pos, len(buf) - 1)   # keep a possibly split marker byte
                                break
                            e = buf.find(EOI, s + 2)
                            if e < 0:
                                pos = s
                                break
                            newest, pos, n = (s, e + 2), e + 2, n + 1
                        if newest:
                            self._publish(bytes(buf[newest[0]:newest[1]]), n - 1)
                        del buf[:pos]
                        if len(buf) > 8_000_000:   # garbage guard
                            buf.clear()
            except Exception as e:
                self.error = str(e)
            self.connected = False
            time.sleep(1.0)


# ---------------------------------------------------------------------------
# 1b. Alternative source: a webcam plugged into THIS laptop (--cam 0)
#     Publishes decoded BGR frames directly (no JPEG round-trip).
# ---------------------------------------------------------------------------
class LocalCameraGrabber(LatestFrameGrabber):
    def __init__(self, index, width=640, height=480, fps=30):
        super().__init__(url=f"local webcam {index}")
        self.index, self.size, self.fps = index, (width, height), fps

    def run(self):
        backend = cv2.CAP_DSHOW if os.name == "nt" else (cv2.CAP_V4L2 if os.path.exists("/dev/video0") else cv2.CAP_ANY)
        while True:
            cap = cv2.VideoCapture(self.index, backend)
            if not cap.isOpened():
                self.error = f"cannot open webcam {self.index}"
                time.sleep(2)
                continue
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.size[0])
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.size[1])
            cap.set(cv2.CAP_PROP_FPS, self.fps)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            self.connected, self.error = True, ""
            fails = 0
            while fails < 30:
                ok, frame = cap.read()      # blocks until the next frame -> always fresh
                if not ok or frame is None:
                    fails += 1
                    time.sleep(0.01)
                    continue
                fails = 0
                self._publish(frame, 0)
            cap.release()
            self.connected = False


# ---------------------------------------------------------------------------
# 2. Image operations
# ---------------------------------------------------------------------------
MORPH_SHAPE = {"rect": cv2.MORPH_RECT, "ellipse": cv2.MORPH_ELLIPSE, "cross": cv2.MORPH_CROSS}
HIST_METRIC = {  # name -> (cv2 flag, higher_is_better)
    "correlation": (cv2.HISTCMP_CORREL, True),
    "intersection": (cv2.HISTCMP_INTERSECT, True),
    "bhattacharyya": (cv2.HISTCMP_BHATTACHARYYA, False),
    "chi-square": (cv2.HISTCMP_CHISQR, False),
}


def odd(k):
    k = max(1, int(k))
    return k if k % 2 else k + 1


def hsv_hist(bgr):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    h = cv2.calcHist([hsv], [0, 1], None, [30, 32], [0, 180, 0, 256])
    return cv2.normalize(h, h, 0, 1, cv2.NORM_MINMAX)


class Gallery:
    """Tiny content-based image retrieval index over ./gallery/*.jpg|png."""

    def __init__(self, folder):
        self.folder = folder
        os.makedirs(folder, exist_ok=True)
        self.items = []   # (name, hist, bgr)
        for f in sorted(os.listdir(folder)):
            if f.lower().endswith((".jpg", ".jpeg", ".png", ".bmp")):
                img = cv2.imread(os.path.join(folder, f))
                if img is not None:
                    self.items.append((f, hsv_hist(img), img))

    def add(self, bgr):
        name = f"snap_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg"
        cv2.imwrite(os.path.join(self.folder, name), bgr)
        self.items.append((name, hsv_hist(bgr), bgr.copy()))
        return name

    def query(self, bgr, metric, k=3):
        if not self.items:
            return []
        flag, higher = HIST_METRIC[metric]
        q = hsv_hist(bgr)
        scored = [(cv2.compareHist(q, h, flag), n, img) for n, h, img in self.items]
        scored.sort(key=lambda t: t[0], reverse=higher)
        return scored[:k]


def put_label(img, text, org=(6, 18), scale=0.5):
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (255, 255, 255), 1, cv2.LINE_AA)


def retrieval_panel(img, gallery, metric):
    H, W = img.shape[:2]
    hh, hw = H // 2, W // 2
    canvas = np.zeros((hh + 64, W, 3), np.uint8)
    canvas[:hh, :hw] = cv2.resize(img, (hw, hh), interpolation=cv2.INTER_AREA)
    put_label(canvas, "query", (4, 14), 0.4)
    results = gallery.query(img, metric)
    if not results:
        put_label(canvas, "gallery empty - click", (hw + 4, 20), 0.4)
        put_label(canvas, "'Add frame to gallery'", (hw + 4, 38), 0.4)
        return canvas, "gallery empty"
    score, name, best = results[0]
    canvas[:hh, hw:hw * 2] = cv2.resize(best, (hw, hh), interpolation=cv2.INTER_AREA)
    put_label(canvas, "best match", (hw + 4, 14), 0.4)
    for i, (s, n, _) in enumerate(results):
        put_label(canvas, f"#{i + 1} {n[:28]}  {s:.3f}", (4, hh + 18 + i * 18), 0.42)
    return canvas, f"top-1: {name} ({score:.3f}) · {len(gallery.items)} in gallery"


PANEL_ORDER = {pn["id"]: i for i, pn in enumerate(PANELS)}


def run_pipeline(frame, p, enabled, gallery, pool=None):
    """Returns ordered list of (panel_id, image, label)."""
    out = []
    want = enabled.__contains__
    H0, W0 = frame.shape[:2]

    # 1. resize -- everything downstream works on this (cheaper = lower latency)
    W = int(p["proc_width"])
    H = max(1, round(H0 * W / W0))
    img = frame if W == W0 else cv2.resize(
        frame, (W, H), interpolation=cv2.INTER_AREA if W < W0 else cv2.INTER_LINEAR)
    if want("original"):
        out.append(("original", frame, f"{W0}×{H0}"))
    if want("resize"):
        out.append(("resize", img, f"{W}×{H}"))

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # 2. color conversion
    if want("color"):
        mode = p["color_mode"]
        conv = {"HSV": cv2.COLOR_BGR2HSV, "LAB": cv2.COLOR_BGR2LAB,
                "YCrCb": cv2.COLOR_BGR2YCrCb, "RGB": cv2.COLOR_BGR2RGB}
        res = gray if mode == "GRAY" else cv2.cvtColor(img, conv[mode])
        out.append(("color", res, f"BGR → {mode} (raw channels shown as B,G,R)" if mode != "GRAY" else "BGR → GRAY"))

    # 3a. Haar detection -- the most expensive op, so it runs on a capped-size
    # image and in parallel with everything below (boxes are scaled back up).
    haar_future = None
    if want("haar"):
        def haar_job():
            dw = min(W, int(p["haar_det_width"]))
            f = W / dw
            small = gray if dw == W else cv2.resize(gray, (dw, max(1, round(H / f))), interpolation=cv2.INTER_AREA)
            dets = get_cascade(p["haar_model"]).detectMultiScale(
                cv2.equalizeHist(small), scaleFactor=float(p["haar_scale"]),
                minNeighbors=int(p["haar_neighbors"]), minSize=(20, 20))
            vis = img.copy()
            for (x, y, w, h) in dets:
                cv2.rectangle(vis, (int(x * f), int(y * f)), (int((x + w) * f), int((y + h) * f)), (0, 255, 0), 2)
            return ("haar", vis, f"{len(dets)} {p['haar_model']} detection(s) · detect @ {dw}px")
        haar_future = pool.submit(haar_job) if pool else None
        if haar_future is None:
            out.append(haar_job())

    # 3b. Otsu (+ mask for morphology)
    morph_ops = [o for o in ("dilate", "erode", "open", "close") if want(o)]
    if want("otsu") or morph_ops:
        src = cv2.GaussianBlur(gray, (5, 5), 0) if p["otsu_blur"] == "gaussian5" else gray
        flag = cv2.THRESH_BINARY_INV if p["otsu_invert"] == "yes" else cv2.THRESH_BINARY
        T, mask = cv2.threshold(src, 0, 255, flag + cv2.THRESH_OTSU)
        if want("otsu"):
            out.append(("otsu", mask, f"threshold T = {T:.0f}"))

    # 4. brightness  g = f + β
    if want("brightness"):
        b = float(p["brightness"])
        out.append(("brightness", cv2.convertScaleAbs(img, alpha=1.0, beta=b), f"β = {b:+.0f}"))

    # 5. contrast  g = α(f - 128) + 128
    if want("contrast"):
        a = float(p["contrast"])
        out.append(("contrast", cv2.convertScaleAbs(img, alpha=a, beta=128 * (1 - a)), f"α = {a:.2f}"))

    # 6. hue / saturation via LUTs on HSV channels
    if want("huesat"):
        shift, scale = int(p["hue_shift"]), float(p["sat_scale"])
        h, s, v = cv2.split(cv2.cvtColor(img, cv2.COLOR_BGR2HSV))
        idx = np.arange(256)
        hue_lut = ((idx + shift) % 180).astype(np.uint8)
        sat_lut = np.clip(idx * scale, 0, 255).astype(np.uint8)
        res = cv2.cvtColor(cv2.merge([cv2.LUT(h, hue_lut), cv2.LUT(s, sat_lut), v]), cv2.COLOR_HSV2BGR)
        out.append(("huesat", res, f"hue {shift:+d}, sat ×{scale:.2f}"))

    # 7. CLAHE on the L channel of LAB (keeps colors)
    if want("clahe"):
        tiles = int(p["clahe_tiles"])
        clahe = cv2.createCLAHE(clipLimit=float(p["clahe_clip"]), tileGridSize=(tiles, tiles))
        l, a_, b_ = cv2.split(cv2.cvtColor(img, cv2.COLOR_BGR2LAB))
        res = cv2.cvtColor(cv2.merge([clahe.apply(l), a_, b_]), cv2.COLOR_LAB2BGR)
        out.append(("clahe", res, f"clip {float(p['clahe_clip']):.1f}, grid {tiles}×{tiles}"))

    # 8. geometric transforms
    if want("translate"):
        tx, ty = float(p["tx"]) / 100 * W, float(p["ty"]) / 100 * H
        M = np.float32([[1, 0, tx], [0, 1, ty]])
        out.append(("translate", cv2.warpAffine(img, M, (W, H)), f"tx={tx:.0f}px, ty={ty:.0f}px"))
    if want("rotate"):
        ang, sc = float(p["angle"]), float(p["rot_scale"])
        M = cv2.getRotationMatrix2D((W / 2, H / 2), ang, sc)
        out.append(("rotate", cv2.warpAffine(img, M, (W, H)), f"{ang:.0f}°, scale {sc:.2f}"))
    if want("flip"):
        code = {"horizontal": 1, "vertical": 0, "both": -1}[p["flip_mode"]]
        out.append(("flip", cv2.flip(img, code), p["flip_mode"]))

    # 9. morphology on the Otsu mask
    for op in morph_ops:
        k, it = odd(p[f"{op}_k"]), int(p[f"{op}_iter"])
        kernel = cv2.getStructuringElement(MORPH_SHAPE[p[f"{op}_shape"]], (k, k))
        if op == "dilate":
            res = cv2.dilate(mask, kernel, iterations=it)
        elif op == "erode":
            res = cv2.erode(mask, kernel, iterations=it)
        else:
            res = cv2.morphologyEx(mask, cv2.MORPH_OPEN if op == "open" else cv2.MORPH_CLOSE,
                                   kernel, iterations=it)
        out.append((op, res, f"{p[f'{op}_shape']} {k}×{k}, iter {it}"))

    # 10. noise removal (optionally inject noise first so the effect is visible)
    if want("denoise"):
        src, inj = img, p["noise_inject"]
        if inj == "salt-pepper":
            src = img.copy()
            r = np.random.random((H, W))
            src[r < 0.03] = 0
            src[r > 0.97] = 255
        elif inj == "gaussian":
            src = cv2.add(img, np.random.normal(0, 20, img.shape).astype(np.int16), dtype=cv2.CV_8U)
        k, m = odd(p["denoise_k"]), p["denoise_method"]
        if m == "median":
            res = cv2.medianBlur(src, k)
        elif m == "gaussian":
            res = cv2.GaussianBlur(src, (k, k), 0)
        else:
            res = cv2.bilateralFilter(src, k, 75, 75)
        if inj != "none":
            res = np.hstack([src, res])
            put_label(res, "noisy", (6, 18))
            put_label(res, "denoised", (W + 6, 18))
        out.append(("denoise", res, f"{m} k={k}" + (f" on {inj} noise" if inj != "none" else "")))

    # 11. image retrieval
    if want("retrieval"):
        res, label = retrieval_panel(img, gallery, p["retrieval_metric"])
        out.append(("retrieval", res, label))

    if haar_future is not None:
        out.append(haar_future.result())
        out.sort(key=lambda t: PANEL_ORDER[t[0]])
    return out, img


# ---------------------------------------------------------------------------
# 3. Processor: newest frame -> all panels -> one packed binary message
# ---------------------------------------------------------------------------
class Processor(threading.Thread):
    def __init__(self, grabber, gallery):
        super().__init__(daemon=True)
        self.grabber, self.gallery = grabber, gallery
        self.lock = threading.Lock()
        self.params = copy.deepcopy(DEFAULT_PARAMS)
        self.snapshot_requested = False
        self.message = None          # (seq, bytes)
        self.pool = ThreadPoolExecutor(max_workers=min(8, os.cpu_count() or 4))
        self._fps_in = self._fps_proc = 0.0
        self.stale = 0

    def update_params(self, values):
        with self.lock:
            for k, v in values.items():
                if k in DEFAULT_PARAMS:
                    self.params[k] = v

    def get_params(self):
        with self.lock:
            return copy.deepcopy(self.params)

    def run(self):
        last_seq, n_proc, t_win = 0, 0, time.time()
        seq_win = 0
        while True:
            jpg, seq, t_recv = self.grabber.wait_new(last_seq, timeout=1.0)
            if jpg is None or seq == last_seq:
                continue
            if last_seq:
                self.stale += seq - last_seq - 1   # arrived while we were busy -> never shown
            last_seq = seq
            try:
                self._process(jpg, seq, t_recv)
            except Exception as e:  # never let a bad param kill the loop
                print("processing error:", repr(e))
            n_proc += 1
            now = time.time()
            if now - t_win >= 1.0:
                self._fps_proc = n_proc / (now - t_win)
                self._fps_in = (seq - seq_win + 0) / (now - t_win)
                seq_win, n_proc, t_win = seq, 0, now

    def _process(self, jpg, seq, t_recv):
        t0 = time.perf_counter()
        if isinstance(jpg, np.ndarray):      # local webcam: already a BGR frame
            frame, in_kb = jpg, 0.0
        else:
            frame, in_kb = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR), len(jpg) / 1024
        if frame is None:
            return
        p = self.get_params()
        orient = p.get("orientation", "none")   # e.g. ESP32-CAM mounted upside down
        if orient != "none":
            frame = cv2.flip(frame, {"rotate 180": -1, "mirror": 1, "flip vertical": 0}[orient])
        enabled = set(p["enabled"])
        t1 = time.perf_counter()
        panels, resized = run_pipeline(frame, p, enabled, self.gallery, self.pool)
        t2 = time.perf_counter()

        if self.snapshot_requested:
            self.snapshot_requested = False
            print("Added to gallery:", self.gallery.add(resized))

        q = [int(cv2.IMWRITE_JPEG_QUALITY), int(p["jpeg_quality"])]
        encoded = list(self.pool.map(lambda item: cv2.imencode(".jpg", item[1], q)[1].tobytes(), panels))
        t3 = time.perf_counter()

        header = {
            "seq": seq,
            "t_recv": t_recv,
            "t_sent": time.time(),
            "stats": {
                "fps_in": round(self._fps_in, 1), "fps_proc": round(self._fps_proc, 1),
                "decode_ms": round((t1 - t0) * 1e3, 1), "process_ms": round((t2 - t1) * 1e3, 1),
                "encode_ms": round((t3 - t2) * 1e3, 1), "skipped": self.grabber.skipped + self.stale,
                "in_kb": round(in_kb, 1), "connected": self.grabber.connected,
            },
            "parts": [{"id": pid, "len": len(b), "label": label}
                      for (pid, _, label), b in zip(panels, encoded)],
        }
        hb = json.dumps(header).encode()
        self.message = (seq, struct.pack("<I", len(hb)) + hb + b"".join(encoded))


# ---------------------------------------------------------------------------
# 4. Web server
# ---------------------------------------------------------------------------
app = FastAPI()
grabber: LatestFrameGrabber = None
processor: Processor = None
ESP32_BASE = None       # e.g. "http://192.168.1.50" when --esp32 is used (for /flash)


def esp32_flash(on):
    try:
        with urllib.request.urlopen(f"{ESP32_BASE}/flash?on={1 if on else 0}", timeout=3) as r:
            return r.read().decode(errors="replace")
    except Exception as e:
        return f"flash request failed: {e}"


@app.get("/")
def index():
    return FileResponse(os.path.join(HERE, "index.html"))


@app.websocket("/ws")
async def ws_endpoint(ws: WebSocket):
    await ws.accept()
    await ws.send_text(json.dumps({"type": "init", "panels": PANELS, "params": processor.get_params(),
                                   "url": grabber.url, "esp32": ESP32_BASE is not None}))
    ready = asyncio.Event()
    closed = False

    async def receiver():
        nonlocal closed
        try:
            while True:
                msg = json.loads(await ws.receive_text())
                t = msg.get("type")
                if t == "ready":
                    ready.set()
                elif t == "params":
                    processor.update_params(msg.get("values", {}))
                elif t == "flash" and ESP32_BASE:
                    res = await asyncio.to_thread(esp32_flash, bool(msg.get("on")))
                    await ws.send_text(json.dumps({"type": "flash", "on": bool(msg.get("on")), "result": res}))
                elif t == "snapshot":
                    processor.snapshot_requested = True
                elif t == "reset":
                    processor.update_params(copy.deepcopy(DEFAULT_PARAMS))
                    await ws.send_text(json.dumps({"type": "params", "params": processor.get_params()}))
        except Exception:
            pass
        finally:
            closed = True
            ready.set()

    rtask = asyncio.create_task(receiver())
    last_seq = -1
    try:
        while not closed:
            await ready.wait()
            if closed:
                break
            msg = processor.message
            if msg is None or msg[0] == last_seq:
                await asyncio.sleep(0.002)
                continue
            ready.clear()
            last_seq = msg[0]
            await ws.send_bytes(msg[1])
    except Exception:
        pass
    finally:
        rtask.cancel()


def main():
    global grabber, processor, ESP32_BASE
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="http://192.168.137.150:5000/video", help="Pi MJPEG stream URL")
    ap.add_argument("--esp32", default=None, metavar="IP",
                    help="ESP32-CAM IP (uses http://IP:81/stream and enables the flash toggle)")
    ap.add_argument("--esp32-port", type=int, default=80, help="ESP32 control port (/flash)")
    ap.add_argument("--esp32-stream-port", type=int, default=81, help="ESP32 stream port (/stream)")
    ap.add_argument("--cam", type=int, default=None,
                    help="use a webcam on THIS laptop instead of the Pi stream (e.g. --cam 0)")
    ap.add_argument("--cam-width", type=int, default=640)
    ap.add_argument("--cam-height", type=int, default=480)
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--gallery", default=os.path.join(HERE, "gallery"), help="folder of reference images for retrieval")
    args = ap.parse_args()

    if not hasattr(cv2, "CascadeClassifier"):
        raise SystemExit(
            f"OpenCV {cv2.__version__}: CascadeClassifier is missing. On OpenCV 5.x it lives in the contrib build:\n"
            "  pip uninstall -y opencv-python opencv-python-headless opencv-contrib-python-headless\n"
            "  pip install opencv-contrib-python")
    try:
        get_cascade("face")          # fetch/load now so a problem shows at startup
    except Exception as e:
        raise SystemExit(f"Cascade setup failed: {e}")
    print(f"OpenCV {cv2.__version__}, cascades from {CASCADE_DIR} / bundled / auto-download")

    if args.cam is not None:
        grabber = LocalCameraGrabber(args.cam, args.cam_width, args.cam_height)
    elif args.esp32:
        ESP32_BASE = f"http://{args.esp32}:{args.esp32_port}"
        grabber = LatestFrameGrabber(f"http://{args.esp32}:{args.esp32_stream_port}/stream")
        print("ESP32-CAM mode: the ESP32 stream serves ONE client at a time -- "
              "close any browser tab showing the ESP32 page.")
    else:
        grabber = LatestFrameGrabber(args.url)
    processor = Processor(grabber, Gallery(args.gallery))
    grabber.start()
    processor.start()
    print(f"Source: {grabber.url}")
    print(f"Open http://localhost:{args.port}")
    uvicorn.run(app, host=args.host, port=args.port, log_level="warning", ws_max_size=16 * 1024 * 1024)


if __name__ == "__main__":
    main()
