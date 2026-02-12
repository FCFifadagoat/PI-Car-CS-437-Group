from picarx import Picarx
import numpy as np
import time


ANGLES = [-20,-10,0,10,20]
SafeDistance = 30
DangerDistance = 20
POWER = 30

forward_angle = 0
grid = np.zeros((100, 100))

car_x_position = 0
car_y_position = 0 

steps = 80

px= Picarx()

def getDistance():
    distance = round(px.ultrasonic.read(), 2)

    # occasionally the ultrasonic sensor returns a value of -2
    # SEE: robot-hat/robot_hat/modules.py line 45-46
    # this is caused by if pulse_start == 0 or pulse_end == 0, which if met, indicates a timeout which can cause unexpected behavior.
    # this can cause some issues with reading distances so if we get it, assume we are in Danger Distance
    while (distance == -2):
        distance = DangerDistance
    return distance

def scanAngles():
    angle_info = []

    for angle in ANGLES:
        px.set_dir_servo_angle(angle)
        time.sleep(0.3)
        
        px.forward(POWER)
        time.sleep(0.3)

        px.stop()
        distance = getDistance()

        px.backward(POWER)
        time.sleep(0.3)
        px.stop()

        if distance < SafeDistance:
            angle_info.append((angle, distance, 1))
        else:
            angle_info.append((angle, distance, 0))
    
    return angle_info

def updateGridWithMapping(angle_info):
    global car_x_position, car_y_position

    prev_obstacle = None

    for angle, distance, is_obstacle in angle_info:
        scan_angle = forward_angle + angle

        if is_obstacle == 1:
            obstacle_x = int(car_x_position + distance * np.cos(np.radians(scan_angle)))
            obstacle_y = int(car_y_position + distance * np.sin(np.radians(scan_angle)))

            if 0 <= obstacle_x < 100 and 0 <= obstacle_y < 100:
                grid[obstacle_y, obstacle_x] = 1

            if prev_obstacle is not None:
                prev_x, prev_y = prev_obstacle
                # - i goes from 0 to steps and i/steps is a percentage from 0 to 1, so we are interpolating between the previous obstacle and the current obstacle and marking all the points in between as obstacles.
                for i in range(steps): 
                    interpolated_x = int(prev_x + (obstacle_x - prev_x) * i / steps)  
                    interpolated_y = int(prev_y + (obstacle_y - prev_y) * i / steps)
                    grid[interpolated_y, interpolated_x] = 1  # mark as obstacle 

            prev_obstacle = (obstacle_x, obstacle_y)

        if is_obstacle == 0:
            free_x = int(car_x_position + distance * np.cos(np.radians(scan_angle)))
            free_y = int(car_y_position + distance * np.sin(np.radians(scan_angle)))

            # - i goes from 0 to steps and i/steps is a percentage from 0 to 1, so we are interpolating between the previous obstacle and the current obstacle and marking all the points in between as obstacles.
            for i in range(steps):
                free_interpolated_x = int(car_x_position + (free_x - car_x_position) * i / steps)
                free_interpolated_y = int(car_y_position + (free_y - car_y_position) * i / steps)

                if 0 <= free_interpolated_x < 100 and 0 <= free_interpolated_y < 100:
                    grid[free_interpolated_y, free_interpolated_x] = 0  # mark as free space 

            prev_obstacle = None

def selectBestDirection(angle_info):
    best_distance = -1
    best_angle = None

    for angle, distance, is_obstacle in angle_info:
        if is_obstacle == 0 and distance > best_distance:
            best_distance = distance
            best_angle = angle

    return best_angle, best_distance

def main():
    try:
        while True:
            angle_info = scanAngles()
            updateGridWithMapping(angle_info)
            best_angle, best_distance = selectBestDirection(angle_info)

            if best_distance < DangerDistance:
                px.backward(POWER)
                time.sleep(0.4)
                px.stop()
                

            px.set_dir_servo_angle(best_angle)
            time.sleep(0.3)
            px.forward(POWER)
            time.sleep(0.3)

    except KeyboardInterrupt:
        print("Stopping car")
        px.stop()


if __name__ == "__main__":
    main()
