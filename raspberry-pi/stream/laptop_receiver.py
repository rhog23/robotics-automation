import cv2
import gradio as gr
import socket
import pickle
import struct
import numpy as np


def process_frame(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    _, segmented = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    return cv2.cvtColor(segmented, cv2.COLOR_GRAY2RGB)


def video_stream():
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    pi_ip = "192.168.137.110"
    port = 9999
    client_socket.connect((pi_ip, port))
    data = b""

    try:
        while True:
            # Receive message size
            while len(data) < 4:  # Hardcode 4 bytes for "L"
                packet = client_socket.recv(4 - len(data))
                if not packet:
                    print("Connection closed by server")
                    break
                data += packet
            msg_size = struct.unpack("L", data[:4])[0]
            data = data[4:]
            print(f"Expected frame size: {msg_size}")

            # Receive frame data
            while len(data) < msg_size:
                remaining = msg_size - len(data)
                packet = client_socket.recv(min(16384, remaining))
                if not packet:
                    print("Incomplete frame received")
                    break
                data += packet

            if len(data) != msg_size:
                print(f"Error: Received {len(data)} bytes, expected {msg_size}")
                break

            frame_data = data[:msg_size]
            data = data[msg_size:]
            print(
                f"Received frame, size: {len(frame_data)}, first 10 bytes: {frame_data[:10]}"
            )

            try:
                frame = pickle.loads(frame_data)
                print("Frame deserialized, shape:", frame.shape)
                segmented_frame = process_frame(frame)
                yield segmented_frame
            except pickle.UnpicklingError as e:
                print(f"Pickle error: {e}, data length: {len(frame_data)}")
                break

            if data:
                print(f"Warning: Leftover data: {len(data)} bytes")
                data = b""

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
