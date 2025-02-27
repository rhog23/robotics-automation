from picamera2 import Picamera2
import socket
import struct
import base64
import time
import cv2

picam2 = Picamera2()
config = picam2.create_video_configuration(
    main={"size": (640, 480), "format": "RGB888"}
)
picam2.configure(config)
picam2.start()
print("Camera started successfully")

time.sleep(2)

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
host_ip = "0.0.0.0"
port = 9999
socket_address = (host_ip, port)

try:
    server_socket.bind(socket_address)
    server_socket.listen(5)
    print(f"Streaming server started on {host_ip}:{port}")

    client_socket, addr = server_socket.accept()
    print(f"Connected to {addr}")

    while True:
        frame = picam2.capture_array()
        if frame is None or frame.size == 0:
            print("Error: Invalid frame captured")
            break

        # raw_data = frame.tobytes()
        encoded_data = cv2.imencode(frame.data)
        # message_size = struct.pack("L", len(encoded_data))

        # client_socket.sendall(message_size + encoded_data)
        client_socket.sendall(encoded_data)

        print("Frame sent successfully")
        time.sleep(0.1)

except Exception as e:
    print(f"Error occurred: {e}")
finally:
    print("Cleaning up...")
    picam2.stop()
    client_socket.close()
    server_socket.close()
