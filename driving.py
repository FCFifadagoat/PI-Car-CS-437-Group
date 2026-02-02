from picarx import Picarx

import time

SafeDistance = 40
DangerDistance = 20
POWER = 50

# function to find the correct direction to proceed in
# takes in the picar (px) so we can control it from within this function
def findGoodAngle(px):
    # start by backing up to make space from the danger
    px.forward(0)
    time.sleep(.3)
    px.backward(POWER)
    time.sleep(.1)
    px.forward(0)

    # turn wheels left and back up, check if distance > DangerDistance
    px.set_dir_servo_angle(-30)
    time.sleep(0.1)
    px.backward(1)
    time.sleep(0.7)
    px.forward(0)
    time.sleep(0.5)

    CheckDistance = round(px.ultrasonic.read(), 2)
    
    # if the new distance is greater than the previous distance, return so we can proceed forwards
    if (CheckDistance > DangerDistance):
        return
    else: # otherwise, reset to original position to try other angle
        px.forward(1)
        time.sleep(0.7)
        px.forward(0)

    # turn right and back up, check if distance > DangerDistance
    px.set_dir_servo_angle(30)
    time.sleep(0.1)
    px.backward(1)
    time.sleep(0.7)
    px.forward(0)

    CheckDistance = round(px.ultrasonic.read(), 2)
    
    # if the new distance is greater than the previous distance, return so we can proceed forwards
    if (CheckDistance > DangerDistance):
        return
    else: # otherwise, reset to original position
        px.forward(1)
        time.sleep(0.7)
        px.forward(0)

    # if still no good angle, back up more and call findGoodAngle again so we can potentially try new angles
    px.backward(POWER)
    time.sleep(0.5)
    px.forward(0)
    findGoodAngle(px)

def main():
    try:
        px = Picarx()

        while True:
            # get the distance value from the ultrasonic sensor
            distance = round(px.ultrasonic.read(), 2)
            # occasionally the ultrasonic sensor returns a value of -2
            # SEE: robot-hat/robot_hat/modules.py line 45-46
            # this is caused by if pulse_start == 0 or pulse_end == 0, which if met, indicates a timeout which can cause unexpected behavior.
            if (distance == -2):
                distance = 40

            # if nothing is within safe distance, drive forwards.
            if distance >= SafeDistance:
                px.set_dir_servo_angle(0)
                px.forward(POWER)

            # if something is within danger distance, stop, back up, turn a bit, and check again, until you get a clear path
            elif distance >= DangerDistance:
                findGoodAngle(px)
                
            # otherwise, back up and try findGoodAngle
            else:
                px.set_dir_servo_angle(0)
                px.backward(1)
                time.sleep(0.5)
                px.forward(0)
                findGoodAngle(px)

    finally:
        px.stop()


if __name__ == "__main__":
    main()