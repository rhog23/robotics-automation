import cv2

cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
if not cap.isOpened():
    print("Error: Could not open camera")
    exit(1)

print("Camera opened successfully")

# Set resolution (optional, to test compatibility)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

for i in range(10):
    ret, frame = cap.read()
    if not ret:
        print(f"Error: Failed to capture frame at attempt {i}")
        break
    print(f"Frame {i} captured, shape: {frame.shape}")
    if i == 0:  # Save the first frame for inspection
        cv2.imwrite("test_frame.jpg", frame)

cap.release()
