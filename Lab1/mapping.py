from picarx import Picarx
import random
import time

SafeDistance = 40
DangerDistance = 20
MoveDistance = 50

def main():
    try:
        px = Picarx()

        while True:
            # get the distance value from the ultrasonic sensor
            distance = round(px.ultrasonic.read(), 2)

            # check if we have any obstacles in front of us
            # if something is within DangerDistance, stop, choose a random direction, back up, turn, and go that direction
            if distance <= DangerDistance:
                px.forward(0)
                time.sleep(0.5)
                angle = random.choice([-30, 30])
                px.set_dir_servo_angle(angle)
                px.backward(MoveDistance * 1.5)
                time.sleep(0.5)
                px.set_dir_servo_angle(0)
                px.forward(MoveDistance)
                time.sleep(0.5)
                
                
            # otherwise, drive straight
            else:
                px.forward(MoveDistance)
                
                

    finally:
        px.stop()


if __name__ == "__main__":
    main()