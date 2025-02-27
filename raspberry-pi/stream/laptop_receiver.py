import cv2
import gradio as gr
import socket
import pickle
import struct
import numpy as np


# Function to process frames with Otsu's method
def process_frame(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, segmented = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    return cv2.cvtColor(segmented, cv2.COLOR_GRAY2RGB)


# Stream receiver function
def video_stream():
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    pi_ip = "192.168.137.110"  # Replace with your Pi's IP address
    port = 9999
    client_socket.connect((pi_ip, port))
    data = b""
    payload_size = struct.calcsize("L")

    try:
        while True:
            # Receive frame size
            while len(data) < payload_size:
                data += client_socket.recv(4096)
            packed_msg_size = data[:payload_size]
            data = data[payload_size:]
            msg_size = struct.unpack("L", packed_msg_size)[0]

            # Receive frame data
            while len(data) < msg_size:
                data += client_socket.recv(4096)
            frame_data = data[:msg_size]
            data = data[msg_size:]

            # Deserialize and process frame
            frame = pickle.loads(frame_data)
            segmented_frame = process_frame(frame)
            yield segmented_frame

    except Exception as e:
        print(f"Stream error: {e}")
    finally:
        client_socket.close()


# Gradio interface
interface = gr.Interface(
    fn=video_stream,
    inputs=None,
    outputs=gr.Image(streaming=True),
    live=True,
    title="Raspberry Pi 4 Otsu Segmentation",
    description="Streaming and segmenting video from Raspberry Pi 4.",
)

interface.launch(server_name="0.0.0.0", server_port=7860)
