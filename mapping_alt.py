from picarx import Picarx
import random
import time

SafeDistance = 40
DangerDistance = 20
MoveDistance = 50



def main():
    try:
        px = Picarx()
        sweepangle = -30

        while True:
            # get the distance value from the ultrasonic sensor
            distance = round(px.ultrasonic.read(), 2)

            # if something is within danger distance, stop, back up, turn a bit, and check again, until you get a clear path
            if distance <= DangerDistance:
                px.forward(0)
                time.sleep(0.3)
                px.backward(MoveDistance)
                time.sleep(0.5)
                px.forward(0)  
                time.sleep(0.3)
                px.set_dir_servo_angle(sweepangle)
                sweepangle += 5
                time.sleep(0.5)
                if sweepangle > 30:
                    sweepangle = -30
            else:
                px.set_dir_servo_angle(sweepangle)
                px.forward(MoveDistance)
                

    finally:
        px.stop()
        px.set_dir_servo_angle(0)


if __name__ == "__main__":
    main()