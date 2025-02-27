import cv2
import gradio as gr
import numpy as np
import urllib.request


def process_frame(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)  # Input is RGB from MJPEG decode
    _, segmented = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    return cv2.cvtColor(segmented, cv2.COLOR_GRAY2RGB)


def video_stream():
    # URL of the Pi's Flask video feed
    url = "http://192.168.137.110:5000/video"
    stream = urllib.request.urlopen(url)
    bytes_data = b""

    while True:
        # Read the stream in chunks
        bytes_data += stream.read(1024)

        # Find JPEG frame boundaries
        start = bytes_data.find(b"\xff\xd8")  # JPEG start
        end = bytes_data.find(b"\xff\xd9")  # JPEG end
        if start != -1 and end != -1:
            jpg = bytes_data[start : end + 2]
            bytes_data = bytes_data[end + 2 :]  # Remove processed frame

            # Decode JPEG to RGB frame
            frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                print("Error: Failed to decode frame")
                continue

            # Process for segmentation
            segmented_frame = process_frame(frame)
            yield segmented_frame


interface = gr.Interface(
    fn=video_stream,
    inputs=None,
    outputs=gr.Image(streaming=True),
    live=True,
    title="Raspberry Pi 4 Otsu Segmentation",
)

interface.launch(server_name="0.0.0.0", server_port=7860)
