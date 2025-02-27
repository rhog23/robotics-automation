import cv2
import gradio as gr
import socket
import struct
import base64
import numpy as np
from io import BytesIO
from PIL import Image


def process_frame(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    _, segmented = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    return cv2.cvtColor(segmented, cv2.COLOR_GRAY2RGB)


def video_stream():
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    pi_ip = "192.168.137.110"
    port = 9999
    client_socket.connect((pi_ip, port))

    try:
        while True:
            # packed_msg_size = b""
            # while len(packed_msg_size) < 4:
            #     packet = client_socket.recv(4 - len(packed_msg_size))
            #     if not packet:
            #         print("Connection closed by server")
            #         break
            #     packed_msg_size += packet
            # msg_size = struct.unpack("L", packed_msg_size)[0]
            # print(f"Expected encoded frame size: {msg_size}")

            # encoded_data = b""
            # while len(encoded_data) < msg_size:
            #     remaining = msg_size - len(encoded_data)
            #     packet = client_socket.recv(min(16384, remaining))
            #     if not packet:
            #         print("Incomplete frame received")
            #         break
            #     encoded_data += packet

            # if len(encoded_data) != msg_size:
            #     print(f"Error: Received {len(encoded_data)} bytes, expected {msg_size}")
            #     break

            # print(f"Received encoded frame, size: {len(encoded_data)}, first 10 bytes: {encoded_data[:10]}")
            packet = client_socket.recv(1024)
            raw_data = Image.frombuffer(
                mode="RGB", size=(480, 640), data=packet
            )
            # print(raw_data)
            # frame = np.frombuffer(raw_data, dtype=np.uint8)
            # print(f"Decoded raw size: {len(raw_data)}")

            # frame = np.fromstring(raw_data, dtype=np.uint8)
            # frame = cv2.imdecode(frame, cv2.IMREAD_ANYCOLOR)
            # frame = Image.open(raw_data)
            # print(f"Frame decoded, shape: {frame.shape}")
            # segmented_frame = process_frame(frame)
            # yield segmented_frame

    except Exception as e:
        print(f"Stream error: {e}")
    finally:
        client_socket.close()


interface = gr.Interface(
    fn=video_stream,
    inputs=None,
    outputs=gr.Image(streaming=True),
    live=True,
    title="Raspberry Pi 4 Otsu Segmentation",
)

interface.launch(server_name="0.0.0.0", server_port=7860)
