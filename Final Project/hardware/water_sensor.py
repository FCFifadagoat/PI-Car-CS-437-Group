import platform
import random

# runs on mac for testing, linux on the pi
MOCK_MODE = platform.system() != "Linux"
BOWL_VOLUME_ML = 500  # average pet bowl

if not MOCK_MODE:
    from gpiozero import DistanceSensor

class WaterSensor:
    def __init__(self, echo_pin=24, trigger_pin=23):
        self.mock_level_ml = 250.0
        if not MOCK_MODE:
            self.sensor = DistanceSensor(echo=echo_pin, trigger=trigger_pin, max_distance=1.0)

    def get_water_level(self):
        if MOCK_MODE:
            self.mock_level_ml += random.uniform(-5, 2.5)
            self.mock_level_ml = max(0, min(BOWL_VOLUME_ML, self.mock_level_ml))
            return round(self.mock_level_ml, 2)
        else:
            # sensor measures distance from top, so closer = more water
            distance = self.sensor.distance
            return round(max(0, BOWL_VOLUME_ML - (distance / 0.2 * BOWL_VOLUME_ML)), 2)
