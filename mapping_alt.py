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
                sweepangle = -30
                found_path = False
                while not found_path:
                    px.set_dir_servo_angle(sweepangle)
                    time.sleep(0.2)
                    checkdistance = round(px.ultrasonic.read(), 2)
                    if checkdistance > SafeDistance:
                        found_path = True
                    else:
                        sweepangle += 10
                        if sweepangle > 30:
                            px.backward(MoveDistance)
                            time.sleep(0.5)
                            sweepangle = -30

                px.forward(MoveDistance)
                time.sleep(1.0)
                px.set_dir_servo_angle(-sweepangle)
                time.sleep(1.0)
                px.set_dir_servo_angle(0) #make sure to calibrate if not working
                time.sleep(0.2)
            # otherwise, drive straight while sweeping the servo to look for obstacles
            else:
                px.set_dir_servo_angle(0)
                px.forward(MoveDistance)
                

    finally:
        px.stop()
        px.set_dir_servo_angle(0)


if __name__ == "__main__":
    main()