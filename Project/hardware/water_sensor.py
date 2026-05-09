import platform
import random

MOCK_MODE = platform.system() != "Linux"

if not MOCK_MODE:
    from gpiozero import DistanceSensor

class WaterSensor:
    def __init__(self, echo_pin=24, trigger_pin=23):
        self.mock_level = 50.0
        if not MOCK_MODE:
            self.sensor = DistanceSensor(echo=echo_pin, trigger=trigger_pin, max_distance=1.0)
            
    def get_water_level(self):
        """Returns the water level as a percentage."""
        if MOCK_MODE:
            self.mock_level += random.uniform(-1, 0.5)
            self.mock_level = max(0, min(100, self.mock_level))
            return round(self.mock_level, 2)
        else:
            # Distance in meters, map to 0-100%
            distance = self.sensor.distance
            # Assuming max distance is 0.2 meters (20cm container)
            percentage = max(0, 100 - (distance / 0.2 * 100))
            return round(percentage, 2)
