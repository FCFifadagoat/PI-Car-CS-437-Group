import socket
import json
from picarx import Picarx

px = Picarx()

HOST = "192.168.0.209"
PORT = 65432

# State dictionary to send back to the frontend
car_state = {
    "distance": 0, 
    "moving": "stopped",
    "speed": 0
}

DEFAULT_SPEED = 50

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    print(f"Server listening on {HOST}:{PORT}")

    try:
        while True:
            client, clientInfo = s.accept()
            data = client.recv(1024)
            
            if data != b"":
                command = data.decode('utf-8').strip()
                
                # Only print non-status commands so we don't spam the terminal every second
                if command != 'status':
                    print(f"Received command: {command}")

                # Map network commands to physical PiCar-X actions
                if command == 'forward':
                    px.set_dir_servo_angle(0)
                    px.forward(DEFAULT_SPEED)
                    car_state['moving'] = 'forward'
                    car_state['speed'] = DEFAULT_SPEED
                    
                elif command == 'backward':
                    px.set_dir_servo_angle(0)
                    px.backward(DEFAULT_SPEED)
                    car_state['moving'] = 'backward'
                    car_state['speed'] = DEFAULT_SPEED
                    
                elif command == 'left':
                    px.set_dir_servo_angle(-30)
                    px.forward(DEFAULT_SPEED)
                    car_state['moving'] = 'turning left'
                    car_state['speed'] = DEFAULT_SPEED
                    
                elif command == 'right':
                    px.set_dir_servo_angle(30)
                    px.forward(DEFAULT_SPEED)
                    car_state['moving'] = 'turning right'
                    car_state['speed'] = DEFAULT_SPEED
                    
                elif command == 'stop':
                    px.forward(0)
                    px.set_dir_servo_angle(0)
                    car_state['moving'] = 'stopped'
                    car_state['speed'] = 0
                    
                elif command == 'status':
                    # Do nothing to the motors. Just let it pass through to 
                    # update the sensor readings below.
                    pass

                # Read the ultrasonic sensor right before sending the response
                raw_distance = px.get_distance()
                car_state['distance'] = round(raw_distance, 2) if raw_distance > 0 else "Error"

                # Send the updated state back to the frontend
                response = json.dumps(car_state)
                client.sendall(response.encode('utf-8'))
                
            client.close()
            
    except Exception as e: 
        print(f"Closing socket due to error: {e}")
        
    finally:
        print("Shutting down motors...")
        px.forward(0)
        s.close()