import time
import platform

MOCK_MODE = platform.system() != "Linux"

if not MOCK_MODE:
    from gpiozero import OutputDevice

class WaterDispenser:
    def __init__(self, relay_pin=17):
        if not MOCK_MODE:
            self.relay = OutputDevice(relay_pin, active_high=False, initial_value=False)
            
    def dispense(self, duration=3):
        """Dispense water for a fixed duration."""
        print(f"Dispensing water for {duration} seconds...")
        if not MOCK_MODE:
            self.relay.on()
            time.sleep(duration)
            self.relay.off()
        else:
            time.sleep(duration)
        print("Done dispensing water.")
