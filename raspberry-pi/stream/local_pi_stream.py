import cv2

cmd = "libcamera-vid -t 0 --inline --nopreview -o - --width 640 --height 480 --codec mjpeg"
cap = cv2.VideoCapture(cmd, cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print("Error: Could not open camera with libcamera")
    exit(1)

print("Camera opened successfully")

ret, frame = cap.read()
if ret:
    print("Frame captured successfully")
    cv2.imwrite("test_frame.jpg", frame)
else:
    print("Error: Failed to capture frame")

cap.release()
