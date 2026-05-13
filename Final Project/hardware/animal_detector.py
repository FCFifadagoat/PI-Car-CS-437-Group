import cv2
import os
import platform

MOCK_MODE = platform.system() != "Linux"

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None

class AnimalDetector:
    def __init__(self, model_path='detection/yolov8n.pt'):
        base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        self.model_path = os.path.join(base_dir, model_path)

        print(f"Initializing AnimalDetector with model: {self.model_path}")
        if YOLO:
            self.model = YOLO(self.model_path)
        else:
            self.model = None
            print("Warning: ultralytics not installed. Detection disabled.")

    def detect_from_frame(self, frame):
        """
        Run detection on a pre-captured frame.
        Returns (animal_type, boxes) where boxes is a list of (x1,y1,x2,y2,name,conf).
        """
        if not self.model:
            return "none", []

        results = self.model.predict(source=frame, classes=[15, 16], conf=0.5, verbose=False)

        boxes = []
        animal_type = "none"
        names = {15: "cat", 16: "dog"}

        if len(results) > 0 and len(results[0].boxes) > 0:
            for box in results[0].boxes:
                class_id = int(box.cls[0])
                if class_id in names:
                    x1, y1, x2, y2 = (int(v) for v in box.xyxy[0])
                    conf = float(box.conf[0])
                    name = names[class_id]
                    boxes.append((x1, y1, x2, y2, name, conf))
                    if animal_type == "none":
                        animal_type = name

        return animal_type, boxes

    def detect_animal(self):
        """
        Captures a single frame from the camera and detects cats or dogs.
        Returns "cat", "dog", or "none". Used as fallback when no shared frame exists.
        """
        if not self.model:
            return "none"

        if MOCK_MODE:
            cap = cv2.VideoCapture(0)
            if not cap.isOpened():
                print("AnimalDetector: Could not open webcam.")
                return "none"
            for _ in range(5):
                cap.read()
            success, frame = cap.read()
            cap.release()
            if not success:
                return "none"
        else:
            try:
                from picamera2 import Picamera2
            except ImportError:
                print("AnimalDetector: picamera2 not found.")
                return "none"
            picam = Picamera2()
            picam.configure(picam.create_preview_configuration(main={"size": (640, 480)}))
            picam.start()
            import time
            time.sleep(0.5)
            frame = picam.capture_array()
            picam.stop()
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        animal_type, _ = self.detect_from_frame(frame)
        return animal_type
