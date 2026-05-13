import os
import json
import platform
import threading
import time
from flask import Flask, render_template, jsonify, request, Response

try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False

# run on mac for testing without hardware
MOCK_MODE = platform.system() != "Linux"

from hardware.water_sensor import WaterSensor
from hardware.water_dispenser import WaterDispenser
from hardware.food_sensor import FoodSensor
from hardware.food_dispenser import FoodDispenser
from hardware.animal_detector import AnimalDetector

app = Flask(__name__)

water_sensor = WaterSensor()
water_dispenser = WaterDispenser()
food_sensor = FoodSensor()
food_dispenser = FoodDispenser()
animal_detector = AnimalDetector()

# how much food to dispense depending on which animal is detected
CAT_FOOD_AMOUNT = 100  # grams
DOG_FOOD_AMOUNT = 300  # grams

MONITOR_INTERVAL = 10       # seconds between monitor loop ticks
LOW_FOOD_RETRY_INTERVAL = 5 # check again sooner if food is low but no animal detected
MAX_HISTORY = 500

last_detected_animal = "none"

# shared state between the camera thread and the main monitor thread
camera_lock = threading.Lock()
camera_state = {
    "frame": None,      # latest raw frame from camera
    "frame_jpg": None,  # latest frame as jpeg with bounding boxes drawn on it
    "boxes": [],        # bounding boxes from last detection run
    "available": False,
}

BOX_COLORS = {"cat": (0, 200, 0), "dog": (0, 140, 255)}  # BGR

DATA_FILE = "data.json"

def load_data():
    if os.path.exists(DATA_FILE):
        with open(DATA_FILE, 'r') as f:
            data = json.load(f)
    else:
        data = {"history": []}

    # make sure consumption keys exist, and handle the old flat format
    c = data.get("consumption", {})
    if "food" not in c and "cat" in c:
        # old format had cat/dog at the top level, migrate it
        c = {"food": {"cat": c.get("cat", 0.0), "dog": c.get("dog", 0.0)}, "water": {"cat": 0.0, "dog": 0.0}}
    c.setdefault("food", {"cat": 0.0, "dog": 0.0})
    c.setdefault("water", {"cat": 0.0, "dog": 0.0})
    data["consumption"] = c
    return data

def save_data(data):
    data["history"] = data["history"][-MAX_HISTORY:]
    with open(DATA_FILE, 'w') as f:
        json.dump(data, f, indent=4)

def dispense_food_until_weight(target_weight, timeout=15):
    if MOCK_MODE:
        food_dispenser.open()
        food_sensor.mock_weight = target_weight + 10
        food_dispenser.close()
        print(f"Mock: food weight set to {food_sensor.mock_weight}g")
        return

    start_time = time.time()
    food_dispenser.open()
    try:
        while True:
            current_weight = food_sensor.get_weight()
            if current_weight >= target_weight:
                print(f"Reached target weight of {target_weight}g")
                break
            if time.time() - start_time > timeout:
                print("Timed out dispensing food, closing to avoid overflow")
                break
            time.sleep(0.5)
    finally:
        food_dispenser.close()

def detect_current_animal():
    global last_detected_animal
    with camera_lock:
        frame = camera_state["frame"]
    if frame is not None:
        animal_type, boxes = animal_detector.detect_from_frame(frame.copy())
        with camera_lock:
            camera_state["boxes"] = boxes
    else:
        # camera thread hasn't started yet, capture a frame directly
        animal_type = animal_detector.detect_animal()
    last_detected_animal = animal_type
    return animal_type

def system_monitor_loop():
    last_food_weight = food_sensor.get_weight()
    last_water_level = water_sensor.get_water_level()

    while True:
        poll_interval = MONITOR_INTERVAL
        try:
            data = load_data()
            dirty = False

            # water 
            water_level = water_sensor.get_water_level()
            water_drop = last_water_level - water_level

            if water_drop > 5.0:
                animal_type = detect_current_animal()
                if animal_type in ["cat", "dog"]:
                    data["consumption"]["water"][animal_type] += water_drop
                    data["history"].append({"timestamp": time.time(), "type": "water_consumption", "animal": animal_type, "amount": water_drop})
                    dirty = True
                    print(f"{animal_type.capitalize()} drank {water_drop:.1f} mL")

            last_water_level = water_level

            if water_level < 100.0:
                print(f"Water low ({water_level:.1f} mL), refilling...")
                water_dispenser.dispense(duration=3)
                last_water_level = water_sensor.get_water_level()
                data["history"].append({"timestamp": time.time(), "type": "water", "action": "auto_dispense"})
                dirty = True

            # food
            current_food_weight = food_sensor.get_weight()
            weight_diff = last_food_weight - current_food_weight

            if weight_diff > 2.0:
                animal_type = detect_current_animal()
                if animal_type in ["cat", "dog"]:
                    data["consumption"]["food"][animal_type] += weight_diff
                    data["history"].append({"timestamp": time.time(), "type": "consumption", "animal": animal_type, "amount": weight_diff})
                    dirty = True
                    print(f"{animal_type.capitalize()} ate {weight_diff:.1f}g")

            last_food_weight = current_food_weight

            if current_food_weight < 50.0:
                animal_type = detect_current_animal()
                if animal_type in ["cat", "dog"]:
                    target_weight = CAT_FOOD_AMOUNT if animal_type == "cat" else DOG_FOOD_AMOUNT
                    print(f"{animal_type.capitalize()} detected, dispensing food to {target_weight}g...")
                    dispense_food_until_weight(target_weight=target_weight)
                    last_food_weight = food_sensor.get_weight()
                    data["history"].append({"timestamp": time.time(), "type": "food", "action": f"auto_dispense_{animal_type}"})
                    dirty = True
                else:
                    # no animal detected yet, check again soon
                    poll_interval = LOW_FOOD_RETRY_INTERVAL

            if dirty:
                save_data(data)

        except Exception as e:
            print(f"Monitor loop error: {e}")

        time.sleep(poll_interval)

def camera_loop():
    if not CV2_AVAILABLE:
        print("opencv not installed, camera disabled")
        return

    if MOCK_MODE:
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("could not open webcam")
            return
        with camera_lock:
            camera_state["available"] = True
        print("camera thread started (webcam)")
        while True:
            success, frame = cap.read()
            if not success:
                time.sleep(0.05)
                continue
            process_camera_frame(frame)
        cap.release()
    else:
        try:
            from picamera2 import Picamera2
        except ImportError:
            print("picamera2 not found")
            return
        picam = Picamera2()
        picam.configure(picam.create_preview_configuration(main={"size": (640, 480)}))
        picam.start()
        with camera_lock:
            camera_state["available"] = True
        print("camera thread started (picamera2)")
        while True:
            frame = picam.capture_array()
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            process_camera_frame(frame)


def process_camera_frame(frame):
    with camera_lock:
        camera_state["frame"] = frame
        boxes = list(camera_state["boxes"])
    display = frame.copy()
    for (x1, y1, x2, y2, name, conf) in boxes:
        color = BOX_COLORS.get(name, (255, 255, 255))
        cv2.rectangle(display, (x1, y1), (x2, y2), color, 2)
        cv2.putText(display, f"{name} {conf:.2f}", (x1, max(y1 - 8, 0)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    _, jpg = cv2.imencode(".jpg", display)
    with camera_lock:
        camera_state["frame_jpg"] = jpg.tobytes()


def generate_mjpeg():
    while True:
        with camera_lock:
            jpg = camera_state["frame_jpg"]
        if jpg is None:
            time.sleep(0.05)
            continue
        yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n"
        time.sleep(0.033)


@app.route('/video_feed')
def video_feed():
    return Response(generate_mjpeg(), mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route('/api/status')
def status():
    with camera_lock:
        cam_available = camera_state["available"]
    data = load_data()
    return jsonify({
        "water_level": water_sensor.get_water_level(),
        "food_weight": food_sensor.get_weight(),
        "last_detected_animal": last_detected_animal,
        "consumption": data["consumption"],
        "camera_available": cam_available,
    })


@app.route('/')
def index():
    return render_template('index.html')

@app.route('/api/dispense/water', methods=['POST'])
def dispense_water():
    duration = request.json.get('duration', 3)
    water_dispenser.dispense(duration)
    data = load_data()
    data["history"].append({"timestamp": time.time(), "type": "water", "action": "dispense"})
    save_data(data)
    return jsonify({"status": "success"})

@app.route('/api/dispense/food', methods=['POST'])
def dispense_food():
    current_weight = food_sensor.get_weight()
    dispense_food_until_weight(target_weight=current_weight + 50.0)
    data = load_data()
    data["history"].append({"timestamp": time.time(), "type": "food", "action": "dispense"})
    save_data(data)
    return jsonify({"status": "success"})

if __name__ == '__main__':
    threading.Thread(target=system_monitor_loop, daemon=True).start()
    threading.Thread(target=camera_loop, daemon=True).start()
    app.run(host='0.0.0.0', port=5001, debug=True, use_reloader=False)
