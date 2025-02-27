import cv2
import socket
import pickle
import struct

# Initialize the camera
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Error: Could not open camera")
    exit(1)

print("Camera opened successfully")

# Set up socket server
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
host_ip = "0.0.0.0"
port = 9999
socket_address = (host_ip, port)

try:
    server_socket.bind(socket_address)
    server_socket.listen(5)
    print(f"Streaming server started on {host_ip}:{port}")

    # Accept a client connection
    client_socket, addr = server_socket.accept()
    print(f"Connected to {addr}")

    # Stream frames
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to capture frame")
            break

        print("Frame captured, sending...")
        data = pickle.dumps(frame)
        message_size = struct.pack("L", len(data))
        client_socket.sendall(message_size + data)
        print("Frame sent")

except Exception as e:
    print(f"Error occurred: {e}")
finally:
    print("Cleaning up...")
    cap.release()
    client_socket.close()
    server_socket.close()
