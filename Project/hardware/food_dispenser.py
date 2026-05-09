import time
import platform

MOCK_MODE = platform.system() != "Linux"

if not MOCK_MODE:
    from gpiozero import Servo

class FoodDispenser:
    def __init__(self, servo_pin=18):
        if not MOCK_MODE:
            self.servo = Servo(servo_pin)
            
    def dispense(self):
        """Dispense food by actuating the servo."""
        print("Actuating servo to dispense food...")
        if not MOCK_MODE:
            self.servo.max()
            time.sleep(1)
            self.servo.min()
            time.sleep(1)
            self.servo.mid() # return to closed
        else:
            time.sleep(2)
        print("Done dispensing food.")
