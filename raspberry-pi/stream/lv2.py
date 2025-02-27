import cv2
import numpy as np
import gradio as gr
import requests
from io import BytesIO

# URL of the video stream from Raspberry Pi (Replace with actual IP)
VIDEO_STREAM_URL = "http://192.168.1.100:5000/video"


def process_frame():
    """Fetches a frame from the Raspberry Pi stream, applies Otsu's thresholding, and returns it."""
    try:
        response = requests.get(VIDEO_STREAM_URL, stream=True, timeout=2)
        if response.status_code == 200:
            image = np.asarray(bytearray(response.raw.read()), dtype=np.uint8)
            frame = cv2.imdecode(image, cv2.IMREAD_COLOR)

            if frame is not None:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                _, segmented = cv2.threshold(
                    gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU
                )
                return segmented
    except Exception as e:
        print(f"Error fetching video stream: {e}")

    return np.zeros((480, 640), dtype=np.uint8)  # Return a blank image on failure


def gradio_interface():
    """Creates and launches the Gradio web app."""
    with gr.Blocks() as app:
        gr.Markdown("# Otsu's Segmentation Web App")
        video_output = gr.Image()

        def update_frame():
            return process_frame()

        gr.Interface(
            fn=update_frame, inputs=[], outputs=video_output, live=True
        ).launch(share=True)


if __name__ == "__main__":
    gradio_interface()
