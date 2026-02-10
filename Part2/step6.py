from picarx import Picarx
import numpy as np
import time


ANGLES = [-20,-10,0,10,20]
THRESHOLD = 30
POWER = 30


forward_angle = 0
grid = np.zeros((100, 100))

car_x_position = 0
car_y_position = 0 

steps = 80

px= Picarx()

#Part 2 , not yet finished.

while True:

    angle_info = []

    for angle in ANGLES:
        px.set_dir_servo_angle(angle)
        time.sleep(0.3)
        px.forward(POWER)
        time.sleep(0.3)
        px.stop()
        distance = px.ultrasonic.read()

        px.backward(POWER)
        time.sleep(0.3)
        px.stop()

        if distance < THRESHOLD:
            angle_info.append((angle,distance,1))
        else:
            angle_info.append((angle,distance,0))

    prev_obstacle = None

    for angle, distance, is_obstacle in angle_info:
        if is_obstacle == 1:
            scan_angle = forward_angle + angle

            obstacle_x = int(car_x_position + distance * np.cos(np.radians(scan_angle)))   # Source : https://www.google.com/search?q=howith+the+distance+and+angle%2C+how+to+get+x%2Cy+coordinates&oq=howith+the+distance+and+angle%2C+how+to+get+x%2Cy+coordinates&gs_lcrp=EgZjaHJvbWUyBggAEEUYOTIJCAEQIRgKGKABMgkIAhAhGAoYoAEyCQgDECEYChigATIJCAQQIRgKGKAB0gEINzM3N2owajeoAgCwAgA&sourceid=chrome&ie=UTF-8 (AI OVverview)
            obstacle_y = int(car_y_position + distance * np.sin(np.radians(scan_angle)))   # Source : https://www.google.com/search?q=howith+the+distance+and+angle%2C+how+to+get+x%2Cy+coordinates&oq=howith+the+distance+and+angle%2C+how+to+get+x%2Cy+coordinates&gs_lcrp=EgZjaHJvbWUyBggAEEUYOTIJCAEQIRgKGKABMgkIAhAhGAoYoAEyCQgDECEYChigATIJCAQQIRgKGKAB0gEINzM3N2owajeoAgCwAgA&sourceid=chrome&ie=UTF-8  (AI Overview)

        
            if 0 <= obstacle_x < 100 and 0 <= obstacle_y < 100:
                grid[obstacle_y, obstacle_x] = 1

            if prev_obstacle is not None:
                    interpolated_y = int(prev_y + (obstacle_y - prev_y) * i / steps) # if i is 0, then interpolated_y will be prev_y which is the start point, and if i is steps, then interpolated_y will be obstacle_y which is end point. So this loop will fill the grid
                    grid[interpolated_y, interpolated_x] = 1
    
            prev_obstacle = obstacle_x, obstacle_y
                
            free_x = int(car_x_position + distance * np.cos(np.radians(scan_angle)))   
            free_y = int(car_y_position + distance * np.sin(np.radians(scan_angle)))   

                if 0 <= free_interpolated_x < 100 and 0 <= free_interpolated_y < 100:
                    grid[free_interpolated_y, free_interpolated_x] = 0
            
            prev_obstacle = None
                            for i in range(steps):
                free_interpolated_x = int(car_x_position + (free_x - car_x_position) * i / steps)
                free_interpolated_y = int(car_y_position + (free_y - car_y_position) * i / steps)
        if is_obstacle == 0:
            scan_angle = forward_angle + angle

                prev_x,prev_y = prev_obstacle

                    interpolated_x = int(prev_x + (obstacle_x - prev_x) * i / steps) # if i is 0, then interpolated_x will be prev_x which is the start point, and if i is steps, then interpolated_x will be obstacle_x which is end point. So this loop will fill the grid 

