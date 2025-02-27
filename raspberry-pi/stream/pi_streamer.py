from picamera2 import Picamera2
from flask import Flask, Response
import cv2
import numpy as np

app = Flask(__name__)

picam2 = Picamera2()
config = picam2.create_video_configuration(
    main={"size": (640, 480), "format": "RGB888"}
)  # Lower resolution
picam2.configure(config)
picam2.start()
print("Camera started successfully")


def generate_frames():
    while True:
        frame = picam2.capture_array()
        if frame is None or frame.size == 0:
            print("Error: Invalid frame captured")
            break

        # Encode with lower quality for smaller size
        ret, buffer = cv2.imencode(
            ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70]
        )  # 70% quality
        if not ret:
            continue

        frame_bytes = buffer.tobytes()
        yield (
            b"--frame\r\n" b"Content-Type: image/jpeg\r\n\r\n" + frame_bytes + b"\r\n"
        )


@app.route("/video")
def video_feed():
    return Response(
        generate_frames(), mimetype="multipart/x-mixed-replace; boundary=frame"
    )


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, threaded=True)
