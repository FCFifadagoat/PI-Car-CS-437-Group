import os
import json
import threading
import time
from flask import Flask, render_template, jsonify, request
from hardware.water_sensor import WaterSensor
from hardware.water_dispenser import WaterDispenser
from hardware.food_sensor import FoodSensor
from hardware.food_dispenser import FoodDispenser

app = Flask(__name__)

water_sensor = WaterSensor()
water_dispenser = WaterDispenser()
food_sensor = FoodSensor()
food_dispenser = FoodDispenser()

DATA_FILE = "data.json"

def load_data():
    if os.path.exists(DATA_FILE):
        with open(DATA_FILE, 'r') as f:
            return json.load(f)
    return {"history": []}

def save_data(data):
    with open(DATA_FILE, 'w') as f:
        json.dump(data, f, indent=4)

def dispense_food_until_weight(target_weight, timeout=15):
    """Dispense food until the target weight is reached or timeout occurs."""
    start_time = time.time()
    food_dispenser.open()
    try:
        while True:
            current_weight = food_sensor.get_weight()
            if current_weight >= target_weight:
                print(f"Target food weight {target_weight}g reached.")
                break
            if time.time() - start_time > timeout:
                print("Dispense food timeout reached. Closing to prevent overflow.")
                break
            time.sleep(0.5)
    finally:
        food_dispenser.close()

def system_monitor_loop():
    while True:
        try:
            # Check water
            water_level = water_sensor.get_water_level()
            if water_level < 20.0:
                print(f"Water level low ({water_level}%). Auto-dispensing water...")
                water_dispenser.dispense(duration=3)
                
                data = load_data()
                data["history"].append({
                    "timestamp": time.time(),
                    "type": "water",
                    "action": "auto_dispense"
                })
                save_data(data)
                
            # Check food
            food_weight = food_sensor.get_weight()
            if food_weight < 50.0:
                print(f"Food weight low ({food_weight}g). Auto-dispensing food to 200g...")
                dispense_food_until_weight(target_weight=200)
                
                data = load_data()
                data["history"].append({
                    "timestamp": time.time(),
                    "type": "food",
                    "action": "auto_dispense"
                })
                save_data(data)
                
        except Exception as e:
            print(f"Error in system monitor: {e}")
            
        time.sleep(300)

monitor_thread = threading.Thread(target=system_monitor_loop, daemon=True)
monitor_thread.start()



@app.route('/')
def index():
    return render_template('index.html')

@app.route('/api/status')
def status():
    return jsonify({
        "water_level": water_sensor.get_water_level(),
        "food_weight": food_sensor.get_weight()
    })

@app.route('/api/dispense/water', methods=['POST'])
def dispense_water():
    duration = request.json.get('duration', 3)
    water_dispenser.dispense(duration)
    
    data = load_data()
    data["history"].append({
        "timestamp": time.time(),
        "type": "water",
        "action": "dispense"
    })
    save_data(data)
    
    return jsonify({"status": "success"})

@app.route('/api/dispense/food', methods=['POST'])
def dispense_food():
    current_weight = food_sensor.get_weight()
    target = current_weight + 50.0 # Dispense 50 more grams manually
    dispense_food_until_weight(target_weight=target)
    
    data = load_data()
    data["history"].append({
        "timestamp": time.time(),
        "type": "food",
        "action": "dispense"
    })
    save_data(data)
    
    return jsonify({"status": "success"})

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5001, debug=True)
