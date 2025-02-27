import cv2
import gradio as gr
import numpy as np
import urllib.request
from ultralytics import YOLO

# Load YOLOv11 model (nano version)
model = YOLO("yolov11n.pt")
print("YOLOv11 model loaded successfully")


def process_frame(frame):
    # Resize frame to reduce inference time
    small_frame = cv2.resize(frame, (320, 240))  # Half the original 640x480
    results = model(small_frame, verbose=False)[0]  # Suppress verbose output
    annotated_frame = results.plot()
    # Resize back to original size for display
    annotated_frame = cv2.resize(annotated_frame, (640, 480))
    return annotated_frame


def video_stream():
    url = "http://192.168.137.110:5000/video"
    stream = urllib.request.urlopen(url)
    bytes_data = b""

    while True:
        bytes_data += stream.read(4096)  # Increase buffer size from 1024
        start = bytes_data.find(b"\xff\xd8")
        end = bytes_data.find(b"\xff\xd9")
        if start != -1 and end != -1:
            jpg = bytes_data[start : end + 2]
            bytes_data = bytes_data[end + 2 :]

            frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                continue

            annotated_frame = process_frame(frame)
            yield annotated_frame


interface = gr.Interface(
    fn=video_stream,
    inputs=None,
    outputs=gr.Image(streaming=True),
    live=True,
    title="Raspberry Pi 4 YOLOv11 Object Detection",
    _js="""() => {
        // Increase Gradio refresh rate
        setInterval(() => {
            document.querySelector('img').src = document.querySelector('img').src;
        }, 100);  // Refresh every 100ms
    }""",
)

interface.launch(server_name="0.0.0.0", server_port=7860, share=False)
