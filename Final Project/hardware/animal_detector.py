import cv2
import os
import platform
import time

MOCK_MODE = platform.system() != "Linux"

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None

# COCO class IDs for the animals we care about
ANIMAL_CLASSES = {15: "cat", 16: "dog"}

class AnimalDetector:
    def __init__(self, model_path='detection/yolov8n.pt'):
        base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        self.model_path = os.path.join(base_dir, model_path)

        print(f"Loading model from {self.model_path}")
        if YOLO:
            self.model = YOLO(self.model_path)
        else:
            self.model = None
            print("ultralytics not installed, detection disabled")

    def detect_from_frame(self, frame):
        if not self.model:
            return "none", []

        results = self.model.predict(source=frame, classes=list(ANIMAL_CLASSES.keys()), conf=0.5, verbose=False)

        boxes = []
        animal_type = "none"

        if results and results[0].boxes:
            for box in results[0].boxes:
                class_id = int(box.cls[0])
                if class_id in ANIMAL_CLASSES:
                    x1, y1, x2, y2 = (int(v) for v in box.xyxy[0])
                    conf = float(box.conf[0])
                    name = ANIMAL_CLASSES[class_id]
                    boxes.append((x1, y1, x2, y2, name, conf))
                    if animal_type == "none":
                        animal_type = name

        return animal_type, boxes

    def detect_animal(self):
        # fallback for when the camera thread hasn't captured a frame yet
        if not self.model:
            return "none"

        if MOCK_MODE:
            cap = cv2.VideoCapture(0)
            if not cap.isOpened():
                print("could not open webcam")
                return "none"
            for _ in range(5):  # flush a few frames so we don't get a stale one
                cap.read()
            success, frame = cap.read()
            cap.release()
            if not success:
                return "none"
        else:
            try:
                from picamera2 import Picamera2
            except ImportError:
                print("picamera2 not found")
                return "none"
            picam = Picamera2()
            picam.configure(picam.create_preview_configuration(main={"size": (640, 480)}))
            picam.start()
            time.sleep(0.5)
            frame = picam.capture_array()
            picam.stop()
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        animal_type, _ = self.detect_from_frame(frame)
        return animal_type
