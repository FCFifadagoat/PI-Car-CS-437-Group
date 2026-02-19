import numpy as np
import time
from picarx import Picarx
from picamera2 import Picamera2
import tflite_runtime.interpreter as tflite
import cv2

MODEL_PATH = "efficientdet_lite0.tflite"
STOP_SIGN_CLASS_ID = 12
MIN_CONFIDENCE = 0.25
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

interpreter = tflite.Interpreter(model_path=MODEL_PATH, num_threads=4)
interpreter.allocate_tensors()

input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

input_height = input_details[0]['shape'][1]
input_width = input_details[0]['shape'][2]

picam2 = Picamera2()
picam2.configure(
    picam2.create_preview_configuration(
        main={"format": "RGB888", "size": (FRAME_WIDTH, FRAME_HEIGHT)}
    )
)
picam2.start()
time.sleep(2)


px = Picarx()

MAP_SIZE = 200
grid = np.zeros((MAP_SIZE, MAP_SIZE))

# the car starts at the center of the grid
# that means, the world coordinate (0, 0) would map to grid[100, 100]
origin_x = MAP_SIZE // 2
origin_y = MAP_SIZE // 2

ANGLES = np.arange(-60, 61, 5) # the angles that the ultrasonic sensor will scan
MIN_DISTANCE = 8 # ignore readings that are too close
MAX_DISTANCE = 80 # ignore readings that are too far


def getDistance():
    readings = []

    for _ in range(3):
        distance = px.ultrasonic.read()
        
        if distance > 0: # Ignore -2 timeout readings and invalid values
            readings.append(distance)

        time.sleep(0.02)

    if len(readings) < 3:
        return -1

    median = np.median(readings)

    # Reject unstable readings
    if np.std(readings) > 10:
        return -1

    return median


# when we detect an object, mark a 3x3 grid around it to create a thicker obstacle region
# we do this because without it, the grid map can sometimes have empty "0" spots where a "1" should be.
def markObstacle(grid_x, grid_y):
    for i in range(-1, 2):
        for j in range(-1, 2):

            x = grid_x + i
            y = grid_y + j

            if 0 <= x < MAP_SIZE and 0 <= y < MAP_SIZE:
                grid[y, x] = 1


def scanEnvironment():
    for angle in ANGLES:

        px.set_cam_pan_angle(angle)
        time.sleep(0.1)

        distance = getDistance()

        if distance == -1: # if distance is -1 (error) then don't add to grid
            continue
        
        if not (MIN_DISTANCE <= distance <= MAX_DISTANCE): # if distance is not in our safe range, then dont add to grid
            continue

        angle_rad = np.deg2rad(angle)

        obj_x = distance * np.cos(angle_rad)
        obj_y = distance * np.sin(angle_rad)

        grid_x = int(origin_x + obj_x)
        grid_y = int(origin_y + obj_y)

        if 0 <= grid_x < MAP_SIZE and 0 <= grid_y < MAP_SIZE:
            markObstacle(grid_x, grid_y)


# helper function to display the grid with the scanned objects
def printGridTerminal():
    # real car dimensions
    car_length = 23
    car_width = 15

    half_length = car_length // 2
    half_width = car_width // 2

    for row in range(MAP_SIZE):
        row_str = ""

        for col in range(MAP_SIZE):
            # check if current cell is inside the car rectangle
            if (origin_x - half_length <= col <= origin_x + half_length and origin_y - half_width <= row <= origin_y + half_width):
                row_str += "C"

            else:
                row_str += str(int(grid[row, col]))

        print(row_str)

def stopSignScanner():
    px.set_cam_pan_angle(0)
    time.sleep(0.2)

    frame = picam2.capture_array()

    resized = cv2.resize(frame, (input_width, input_height))
    input_data = np.expand_dims(resized, axis=0)

    if input_details[0]['dtype'] == np.float32:
        input_data = (np.float32(input_data) - 127.5) / 127.5

    interpreter.set_tensor(input_details[0]['index'], input_data)
    interpreter.invoke()

    boxes = interpreter.get_tensor(output_details[0]['index'])[0]
    classes = interpreter.get_tensor(output_details[1]['index'])[0]
    scores = interpreter.get_tensor(output_details[2]['index'])[0]
    num_detections = int(interpreter.get_tensor(output_details[3]['index'])[0])

    height, width, _ = frame.shape

    for i in range(num_detections):

        if scores[i] > MIN_CONFIDENCE:
            class_id = int(classes[i])
            score = scores[i]

            print(f"Detected class {class_id} score {score:.2f}")
            
            if class_id == STOP_SIGN_CLASS_ID:
                ymin, xmin, ymax, xmax = boxes[i]

                x1 = int(xmin * width)
                y1 = int(ymin * height)
                x2 = int(xmax * width)
                y2 = int(ymax * height)

                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                current_time = time.time()
                if current_time - last_stop_time > STOP_COOLDOWN:
                    print(f"Detected STOP SIGN score {score:.2f}. Stopping!")
                    px.stop()
                    time.sleep(2)
                    px.forward(20)
                    last_stop_time = time.time() # Reset the cooldown timer
                else:
                    print("Stop sign detected, but still in cooldown.")





def main():
    try:
        while True:
            scanEnvironment()
            printGridTerminal()
            time.sleep(0.1)
            stopSignScanner()
            

    except KeyboardInterrupt:
        print("Stopping car")
        px.set_cam_pan_angle(0)
        px.stop()
        picam2.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()