import socket
import json

UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", UDP_PORT))
sock.setblocking(True)
sock.settimeout(None)
print(f"Listening on UDP port {UDP_PORT}")

while True:
    data, addr = sock.recvfrom(1024)
    decoded = json.loads(data.decode())
    print("UDP Received from", addr, decoded)
