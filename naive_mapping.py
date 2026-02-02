from picarx import Picarx
import numpy as np
import time


ANGLES = [-20,-10,0,10,20]
THRESHOLD = 30 

def main():
    px = Picarx()

    # 1: Obstacle, 0 : No obstacle
    obstacle_map = np.zeros(len(ANGLES))
    for i, angle in enumerate(ANGLES):
        px.set_dir_servo_angle(angle)
        time.sleep(0.3)
        distance = px.ultrasonic.read()

        if distance < THRESHOLD:
            obstacle_map[i] = 1
        else:
            obstacle_map[i] = 0

    px.set_dir_servo_angle(0)

if __name__ == "__main__":
    main()
