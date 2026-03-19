import socket, sys

if len(sys.argv) < 3:
    sys.exit()

command, mac = sys.argv[1], sys.argv[2]

try:
    s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
    s.connect((mac, 1))
    s.send(command.encode())
    print(s.recv(1024).decode())
    s.close()
except Exception as e:
    import json
    print(json.dumps({"error": str(e)}))