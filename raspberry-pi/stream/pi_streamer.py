from picamera2 import Picamera2
import socket
import pickle
import struct
import time

# Initialize PiCamera2
picam2 = Picamera2()
config = picam2.create_video_configuration(main={"size": (640, 480), "format": "RGB888"})
picam2.configure(config)
picam2.start()
print("Camera started successfully")

# Give the camera a moment to warm up
time.sleep(2)  # Adjust if needed

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
        # Capture frame as a NumPy array
        frame = picam2.capture_array()
        if frame is None:
            print("Error: Failed to capture frame")
            break
        
        print("Frame captured, shape:", frame.shape)
        data = pickle.dumps(frame)
        message_size = struct.pack("L", len(data))
        client_socket.sendall(message_size + data)
        print("Frame sent")
        time.sleep(0.01)  # Small delay for stability

except Exception as e:
    print(f"Error occurred: {e}")
finally:
    print("Cleaning up...")
    picam2.stop()
    client_socket.close()
    server_socket.close()