import time
import cv2
import numpy as np
from picamera2 import Picamera2
import tflite_runtime.interpreter as tflite

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

print("Stop Sign Detection Running...")

while True:

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
                print("STOP SIGN DETECTED!")

                ymin, xmin, ymax, xmax = boxes[i]

                x1 = int(xmin * width)
                y1 = int(ymin * height)
                x2 = int(xmax * width)
                y2 = int(ymax * height)

                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

picam2.stop()
cv2.destroyAllWindows()