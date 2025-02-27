import cv2
import gradio as gr
import numpy as np
import urllib.request
from ultralytics import YOLO
import time

# Load YOLOv11 model (nano version)
model = YOLO("yolo11n.pt")
print("YOLOv11 model loaded successfully")


def process_frame(frame):
    # Resize for faster inference
    small_frame = cv2.resize(frame, (320, 240))
    results = model(small_frame, verbose=False)[0]
    annotated_frame = results.plot()
    # Resize back for display
    # return cv2.resize(annotated_frame, (640, 480))
    return annotated_frame


def video_stream():
    url = "http://192.168.137.110:5000/video"
    stream = urllib.request.urlopen(url)
    bytes_data = b""
    frame_count = 0

    while True:
        bytes_data += stream.read(4096)
        start = bytes_data.find(b"\xff\xd8")
        end = bytes_data.find(b"\xff\xd9")
        if start != -1 and end != -1:
            jpg = bytes_data[start : end + 2]
            bytes_data = bytes_data[end + 2 :]

            frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                continue

            # Skip every other frame to reduce load
            frame_count += 1
            if frame_count % 2 == 0:  # Process every 2nd frame
                annotated_frame = process_frame(frame)
            else:
                annotated_frame = frame  # Pass raw frame

            # Log FPS
            current_time = time.time()
            fps = (
                1 / (current_time - video_stream.last_time)
                if hasattr(video_stream, "last_time")
                else 0
            )
            video_stream.last_time = current_time
            print(f"FPS: {fps:.2f}")

            yield annotated_frame


# Initialize last_time
video_stream.last_time = time.time()

interface = gr.Interface(
    fn=video_stream,
    inputs=None,
    outputs=gr.Image(streaming=True),
    live=True,
    title="Raspberry Pi 4 YOLOv11 Object Detection",
)

interface.launch(server_name="0.0.0.0", server_port=7860, share=False)
