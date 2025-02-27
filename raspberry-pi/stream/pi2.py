from flask import Flask, Response
from picamera2 import Picamera2
import cv2
import numpy as np

app = Flask(__name__)

# Initialize PiCamera2
camera = Picamera2()
config = camera.create_video_configuration(
    main={"size": (640, 480), "format": "RGB888"}
)
camera.configure(config)
camera.start()


def generate_frames():
    while True:
        frame = camera.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)  # Convert from BGR to RGB
        ret, buffer = cv2.imencode(".jpg", frame)
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
    app.run(host="0.0.0.0", port=5000, debug=True)
