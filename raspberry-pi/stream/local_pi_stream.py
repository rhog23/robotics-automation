import cv2
import gradio as gr
import subprocess
import numpy as np


def process_frame(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, segmented = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    return cv2.cvtColor(segmented, cv2.COLOR_GRAY2RGB)


def video_stream():
    # Use libcamera to capture frames (adjust path if needed)
    cmd = "libcamera-vid -t 0 --inline --nopreview -o -"
    process = subprocess.Popen(cmd.split(), stdout=subprocess.PIPE)

    while True:
        raw_frame = process.stdout.read(640 * 480 * 3)  # Adjust resolution as needed
        if not raw_frame:
            break
        frame = np.frombuffer(raw_frame, np.uint8).reshape(480, 640, 3)
        segmented_frame = process_frame(frame)
        yield segmented_frame


interface = gr.Interface(
    fn=video_stream, inputs=None, outputs=gr.Image(streaming=True), live=True
)

interface.launch(server_name="0.0.0.0", server_port=7860)
