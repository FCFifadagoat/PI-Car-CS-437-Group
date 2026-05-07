from picarx import Picarx
import time

px = Picarx()
SafeDistance = 30
DangerDistance = 10 #testing lower DangerDistance 
POWER = 35 # test value 

# Test Committing turn time to try and clear obstacle more clearly 
MinDistance = 35    
BackupDistance = 0.5
CommitTurn = 1.0     

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
    # Back up further to get better readings
    px.forward(0)
    time.sleep(.3)
    px.backward(POWER)
    time.sleep(BackupDistance)  # Increased backup time
    px.forward(0)
    time.sleep(0.3)
    
    # turn wheels left, back up, and scan
    px.set_dir_servo_angle(-30)
    time.sleep(0.)
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
    time.sleep(0.2)
    px.backward(POWER)
    time.sleep(0.8)
    px.forward(0)
    time.sleep(0.5)
    LeftDistance = getDistance()
    
    print(f"Left:{LeftDistance} | Right: {RightDistance}")
    
    # Test best clearance angles 
    if LeftDistance < MinDistance and RightDistance < MinDistance:
        # Pathing is blocked
        print("Path is blocked. Backing up")
        px.backward(POWER)
        time.sleep(0.8)
        px.forward(0)
        return
    
    # if the left distance is greater than the right distance, commit to left turn
    if (LeftDistance >= RightDistance):
        print(f"Going left - clearance: {LeftDistance}cm")
        # Already positioned right, so continue forward while turned
        px.forward(POWER)
        time.sleep(CommitTurn)  # Commit to the turn longer
        px.forward(0)
        return
    else:
        # reset to original position, turn left, backup, and commit to right turn
        print(f"Going right - clearance: {RightDistance}cm")
        px.forward(POWER)
        time.sleep(0.8)
        px.forward(0)
        # turn wheels left and back up
        px.set_dir_servo_angle(-30)
        time.sleep(0.2)
        px.backward(POWER)
        time.sleep(0.8)
        px.forward(0)
        # Then commit to moving forward in this direction
        px.forward(POWER)
        time.sleep(CommitTurn)
        px.forward(0)
        return

def main():
    try:
        while True:
            # get the distance value from the ultrasonic sensor
            distance = getDistance()
            
            # check angle inside safe distance
            if distance < SafeDistance:
                print(f"Obstacle detected at {distance}cm. Changing course ...")
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
