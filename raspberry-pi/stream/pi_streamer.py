import cv2
import socket
import pickle
import struct

# Initialize the Pi Camera
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    raise RuntimeError("Could not open camera")

# Set up socket server
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
host_ip = "0.0.0.0"  # Listen on all interfaces
port = 9999
socket_address = (host_ip, port)
server_socket.bind(socket_address)
server_socket.listen(5)
print(f"Streaming server started on {host_ip}:{port}")

# Accept a client connection (your laptop)
client_socket, addr = server_socket.accept()
print(f"Connected to {addr}")

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Serialize frame
        data = pickle.dumps(frame)
        message_size = struct.pack("L", len(data))  # Pack the size of the data

        # Send frame size and data
        client_socket.sendall(message_size + data)

except Exception as e:
    print(f"Error: {e}")
finally:
    cap.release()
    client_socket.close()
    server_socket.close()
