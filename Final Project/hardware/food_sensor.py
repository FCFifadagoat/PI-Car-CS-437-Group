import platform
import random

MOCK_MODE = platform.system() != "Linux"

if not MOCK_MODE:
    # Requires HX711 library
    try:
        from hx711 import HX711
    except ImportError:
        MOCK_MODE = True
        print("HX711 library not found. Falling back to mock mode.")

class FoodSensor:
    def __init__(self, dout_pin=5, pd_sck_pin=6):
        self.mock_weight = 200.0
        if not MOCK_MODE:
            self.hx = HX711(dout_pin, pd_sck_pin)
            self.hx.set_reading_format("MSB", "MSB")
            self.hx.set_reference_unit(114) # Needs calibration - can be found using raw value/known weight 
            self.hx.reset()
            self.hx.tare()
            
    def get_weight(self):
        """Returns the food weight in grams."""
        if MOCK_MODE:
            self.mock_weight += random.uniform(-5, 2)
            self.mock_weight = max(0, self.mock_weight)
            return round(self.mock_weight, 2)
        else:
            val = self.hx.get_weight(5)
            self.hx.power_down()
            self.hx.power_up()
            return round(val, 2)
