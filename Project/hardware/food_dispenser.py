import time
import platform

MOCK_MODE = platform.system() != "Linux"

if not MOCK_MODE:
    from gpiozero import Servo

class FoodDispenser:
    def __init__(self, servo_pin=18):
        if not MOCK_MODE:
            self.servo = Servo(servo_pin)
            
    def open(self):
        """Open the food dispenser."""
        print("Opening food dispenser...")
        if not MOCK_MODE:
            self.servo.max()

    def close(self):
        """Close the food dispenser."""
        print("Closing food dispenser...")
        if not MOCK_MODE:
            self.servo.mid() # return to closed
