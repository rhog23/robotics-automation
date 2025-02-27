import cv2
from picamera2 import Picamera2

picam2 = Picamera2()
picam2.configure(
    picam2.create_preview_configuration(main={"format": "XRGB8888", "size": (640, 480)})
)
picam2.start()

try:
    while True:
        # Capture a frame from the camera
        frame = picam2.capture_array()

        # Display the frame using OpenCV
        cv2.imshow("Camera Preview", frame)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
finally:
    picam2.stop()
    cv2.destroyAllWindows()
