import cv2
import numpy as np
import urllib.request
from ultralytics import YOLO
import time

# Load YOLOv11 model (nano version)
model = YOLO("yolo11n.pt")
print("YOLOv11 model loaded successfully")


def process_frame(frame):
    # Resize for faster inference
    small_frame = cv2.resize(frame, (320, 240))
    results = model(small_frame, verbose=False, stream=True)
    # annotated_frame = results.plot()
    # Resize back for display
    # return cv2.resize(annotated_frame, (640, 480))
    # return cv2.resize(annotated_frame, (320, 240))
    for result in results:
        for box in result.boxes:
            box = box.xyxy
            x = int(box[0][0])
            y = int(box[0][1])
            w = int(box[0][2])
            h = int(box[0][3])

            cv2.rectangle(small_frame, (x, y), (w, h), (0, 0, 0), 2)
    return small_frame


def video_stream():
    url = "http://192.168.137.110:5000/video"
    stream = urllib.request.urlopen(url)
    bytes_data = b""
    frame_count = 0
    last_time = time.time()

    while True:
        bytes_data += stream.read(4096)
        start = bytes_data.find(b"\xff\xd8")
        end = bytes_data.find(b"\xff\xd9")
        if start != -1 and end != -1:
            jpg = bytes_data[start : end + 2]
            bytes_data = bytes_data[end + 2 :]

            frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                continue

            # Skip every other frame to reduce load
            frame_count += 1
            if frame_count % 2 == 0:  # Process every 2nd frame
                annotated_frame = process_frame(frame)
            else:
                annotated_frame = frame

            # Calculate FPS
            current_time = time.time()
            fps = 1 / (current_time - last_time)
            last_time = current_time

            # Overlay FPS on the frame
            cv2.putText(
                annotated_frame,
                f"FPS: {fps:.2f}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 0),
                2,
            )

            # Display the frame
            cv2.imshow("Raspberry Pi YOLOv11 Detection", annotated_frame)

            # Break on 'q' key press
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break


if __name__ == "__main__":
    try:
        video_stream()
    except Exception as e:
        print(f"Error: {e}")
    finally:
        cv2.destroyAllWindows()
