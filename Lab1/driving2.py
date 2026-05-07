from picarx import Picarx

import time

px = Picarx()

SafeDistance = 30
DangerDistance = 20
POWER = 35

# function to get the distance using ultrasonic sensor
def getDistance():
    distance = round(px.ultrasonic.read(), 2)

    # occasionally the ultrasonic sensor returns a value of -2
    # SEE: robot-hat/robot_hat/modules.py line 45-46
    # this is caused by if pulse_start == 0 or pulse_end == 0, which if met, indicates a timeout which can cause unexpected behavior.
    # this can cause some issues with reading distances so if we get it, assume we are in Danger Distance
    while (distance == -2):
        distance = DangerDistance
    return distance


# function to find the correct direction to proceed in
def findGoodAngle():
    # 1) Obstacle detected → stop first (stabilize)
    px.forward(0)
    time.sleep(0.3)
    px.backward(POWER)
    time.sleep(0.8)
    px.forward(0)
    time.sleep(0.3)

    # Left Scan
    # Turn wheels left -> move slightly -> stop -> measure distance
    px.set_dir_servo_angle(-30)
    time.sleep(0.3)

    px.forward(20)
    time.sleep(0.4)

    px.forward(0)
    time.sleep(0.3)

    LeftDistance = getDistance()

    # Return to original position after left scan
    px.backward(20)
    time.sleep(0.4)

    px.forward(0)
    time.sleep(0.3)

    # Right Scan
    # Turn wheels right -> move slightly -> stop -> measure distance
    px.set_dir_servo_angle(30)    
    time.sleep(0.3)

    px.forward(20)
    time.sleep(0.4)

    px.forward(0)
    time.sleep(0.3)

    RightDistance = getDistance()

    # Return to original positin after right scan
    px.backward(20)
    time.sleep(0.4)

    px.forward(0)
    time.sleep(0.3)

    print(f"Left: {LeftDistance} | Right: {RightDistance}")

    # Move to the safer side based on left&right distance
    if LeftDistance >= RightDistance:
        px.set_dir_servo_angle(-30)
        time.sleep(0.3)

        px.forward(POWER)
        time.sleep(0.6)

    else:
        px.set_dir_servo_angle(30)
        time.sleep(0.3)

        px.forward(POWER)
        time.sleep(0.6)

    # Finally, we center the wheels
    px.set_dir_servo_angle(0)
    time.sleep(0.2)


def main():
    try:
        while True:
            # get the distance value from the ultrasonic sensor
            distance = getDistance()

            # if something is inside of the safe distance, stop and check for a good angle to turn to
            if distance < SafeDistance:
                findGoodAngle()

            # otherwise, drive straight
            else:
                px.set_dir_servo_angle(0)
                px.forward(POWER)

    except KeyboardInterrupt:
        print("Stopping car")
        px.stop()


if __name__ == "__main__":
    main()
