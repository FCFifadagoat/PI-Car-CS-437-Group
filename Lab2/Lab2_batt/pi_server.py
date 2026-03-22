import socket, json, threading
from picarx import Picarx
from robot_hat import ADC

px = Picarx()
car_state = {"distance": 0, "moving": "stopped", "speed": 0}
battery_adc = ADC('A4')

def handle(cmd):
    if cmd == 'forward':
        px.set_dir_servo_angle(0); px.forward(50)
        car_state.update({"moving": "forward", "speed": 50})
    elif cmd == 'backward':
        px.set_dir_servo_angle(0); px.backward(50)
        car_state.update({"moving": "backward", "speed": 50})
    elif cmd == 'left':
        px.set_dir_servo_angle(-30); px.forward(50)
        car_state.update({"moving": "left", "speed": 50})
    elif cmd == 'right':
        px.set_dir_servo_angle(30); px.forward(50)
        car_state.update({"moving": "right", "speed": 50})
    elif cmd == 'stop':
        px.forward(0); px.set_dir_servo_angle(0)
        car_state.update({"moving": "stopped", "speed": 0})
    
    car_state["distance"] = round(px.get_distance(), 2)
    
    try:
        raw_reading_battery = battery_adc.read()
        voltage = raw_reading_battery / 4095 * 3.3 * 3
        percentage = round((voltage - 6.0) / (7.4-6.0) * 100, 1)
        percentage = max(0, min(100, percentage))
        car_state["battery"] = percentage
    except Exeception as e:
        print(f"Battery error: {e}")
        car_state["battery"] = "No Battery Level Acquired"
        
    return json.dumps(car_state)

def wifi_loop():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("0.0.0.0", 65432)); s.listen()
    while True:
        try:
            c, _ = s.accept()
            msg = c.recv(1024).decode().strip()
            if msg: c.send(handle(msg).encode())
            c.close()
        except: pass

def bt_loop():
    s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
    s.bind(("88:A2:9E:3b:04:AF", 1)); s.listen(1)
    while True:
        try:
            c, _ = s.accept()
            msg = c.recv(1024).decode().strip()
            if msg: c.send(handle(msg).encode())
            c.close()
        except: pass

threading.Thread(target=wifi_loop, daemon=True).start()
threading.Thread(target=bt_loop, daemon=True).start()

print("Servers Running (WiFi & BT)... Ctrl+C to exit")
try:
    while True: threading.Event().wait(1)
except:
    px.forward(0)
