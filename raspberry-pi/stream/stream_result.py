import cv2
import numpy as np
import urllib.request
from ultralytics import YOLO
import time
import threading
import queue

# Load YOLOv11 model (nano version)
model = YOLO("yolo11n_openvino_model", task="detect")
print("YOLOv11 model loaded successfully")

# Queue to hold decoded frames
frame_queue = queue.Queue(maxsize=5)  # Limit to 5 frames to avoid memory overload


def process_frame(frame):
    # Resize for faster inference
    # small_frame = cv2.resize(frame, (320, 240))
    results = model(frame, verbose=False)[0]
    annotated_frame = results.plot()
    # return cv2.resize(annotated_frame, (640, 480))
    return annotated_frame


def fetch_frames():
    url = "http://192.168.137.110:5000/video"
    stream = urllib.request.urlopen(url)
    bytes_data = b""

    while True:
        try:
            bytes_data += stream.read(4096)
            start = bytes_data.find(b"\xff\xd8")
            end = bytes_data.find(b"\xff\xd9")
            if start != -1 and end != -1:
                jpg = bytes_data[start : end + 2]
                bytes_data = bytes_data[end + 2 :]

                frame = cv2.imdecode(
                    np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR
                )
                if frame is not None:
                    # Add to queue, skip if full
                    try:
                        frame_queue.put_nowait(frame)
                    except queue.Full:
                        pass  # Drop frame if queue is full
        except Exception as e:
            print(f"Fetch error: {e}")
            break


def video_stream():
    # Start the frame-fetching thread
    fetch_thread = threading.Thread(target=fetch_frames, daemon=True)
    fetch_thread.start()

    last_time = time.time()
    frame_count = 0

    while True:
        try:
            # Get latest frame from queue (non-blocking)
            frame = frame_queue.get(timeout=1.0)  # Wait up to 1s for a frame
            frame_queue.task_done()

            # Skip every other frame to reduce load
            frame_count += 1
            if frame_count % 2 == 0:
                annotated_frame = process_frame(frame)
            else:
                annotated_frame = frame

            # Calculate FPS
            current_time = time.time()
            fps = 1 / (current_time - last_time)
            last_time = current_time

            # Overlay FPS
            cv2.putText(
                annotated_frame,
                f"FPS: {fps:.2f}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 0),
                2,
            )

            # Display
            cv2.imshow("Raspberry Pi YOLOv11 Detection", annotated_frame)

            # Quit on 'q'
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

        except queue.Empty:
            print("Queue empty, waiting for frames...")
            continue
        except Exception as e:
            print(f"Processing error: {e}")
            break


if __name__ == "__main__":
    try:
        video_stream()
    except KeyboardInterrupt:
        print("Stopped by user")
    finally:
        cv2.destroyAllWindows()
