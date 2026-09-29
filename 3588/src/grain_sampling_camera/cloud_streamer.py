#!/usr/bin/env python3
"""Cloud video streamer — captures camera frames and POSTs to cloud API."""
from __future__ import annotations
import argparse, logging, time, urllib.request, json, os

logger = logging.getLogger("cloud_streamer")
CLOUD_URL = os.environ.get("CLOUD_VIDEO_URL", "http://cloud-api.xxx.com/video/upload")
DEVICE = int(os.environ.get("CAMERA_DEVICE", "0"))
FPS = float(os.environ.get("CLOUD_STREAM_FPS", "2"))
JPEG_QUALITY = int(os.environ.get("CLOUD_JPEG_QUALITY", "50"))
WIDTH, HEIGHT = 640, 480

try:
    import cv2
except ImportError:
    cv2 = None
    logger.error("opencv-python not installed")

def main():
    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
    if cv2 is None:
        logger.error("cv2 not available, exiting")
        return

    cap = cv2.VideoCapture(DEVICE)
    if not cap.isOpened():
        logger.error("Cannot open /dev/video%d", DEVICE)
        return
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    logger.info("Cloud streamer started: %dx%d @ %.1f FPS -> %s", WIDTH, HEIGHT, FPS, CLOUD_URL)

    interval = 1.0 / FPS
    while True:
        ok, frame = cap.read()
        if not ok:
            time.sleep(1)
            continue
        _, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        try:
            req = urllib.request.Request(
                CLOUD_URL, data=jpeg.tobytes(),
                headers={"Content-Type": "image/jpeg", "X-Robot-Mac": "unknown"}
            )
            urllib.request.urlopen(req, timeout=5)
        except Exception as e:
            logger.debug("Upload skip: %s", e)
        time.sleep(interval)

if __name__ == "__main__":
    main()
