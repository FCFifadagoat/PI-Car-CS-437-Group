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

def water_monitor_loop():
    while True:
        try:
            level = water_sensor.get_water_level()
            if level < 20.0:
                print(f"Water level low ({level}%). Auto-dispensing water...")
                water_dispenser.dispense(duration=3)
                
                data = load_data()
                data["history"].append({
                    "timestamp": time.time(),
                    "type": "water",
                    "action": "auto_dispense"
                })
                save_data(data)
        except Exception as e:
            print(f"Error in water monitor: {e}")
            
        time.sleep(300)

monitor_thread = threading.Thread(target=water_monitor_loop, daemon=True)
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
    food_dispenser.dispense()
    
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
