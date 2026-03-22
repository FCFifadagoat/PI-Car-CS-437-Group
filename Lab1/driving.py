from picarx import Picarx

import time

px = Picarx()

SafeDistance = 30
DangerDistance = 20
POWER = 50

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
    # start by backing up to make space from the danger
    px.forward(0)
    time.sleep(.3)
    px.backward(POWER)
    time.sleep(.1)
    px.forward(0)

    # turn wheels left, back up, and scan
    px.set_dir_servo_angle(-30)
    time.sleep(0.1)
    px.backward(POWER)
    time.sleep(0.8)
    px.forward(0)
    time.sleep(0.5)

    RightDistance = getDistance()
    
    # reset to original position to check other angle
    px.forward(POWER)
    time.sleep(0.8)
    px.forward(0)
    time.sleep(0.5)

    # turn wheels right, back up, and scan
    px.set_dir_servo_angle(30)
    time.sleep(0.1)
    px.backward(POWER)
    time.sleep(0.8)
    px.forward(0)
    time.sleep(0.5)

    LeftDistance = getDistance()
    
    print(f"Left:{LeftDistance} | Right: {RightDistance}")
    # if the left distance is greater than the right distance, return so we can proceed forwards
    if (LeftDistance >= RightDistance):
        px.set_dir_servo_angle(0)
        return
    else: # otherwise, reset to original position, turn left, backup, and return
        px.forward(POWER)
        time.sleep(0.8)
        px.forward(0)
        # turn wheels left and back up
        px.set_dir_servo_angle(-30)
        time.sleep(0.1)
        px.backward(POWER)
        time.sleep(0.8)
        px.forward(0)
        time.sleep(0.5)
        return


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
