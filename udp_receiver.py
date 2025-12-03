import socket
import json

UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", UDP_PORT))
sock.setblocking(True)
sock.settimeout(None)

def get_udp_data():
        data, addr = sock.recvfrom(1024)
        decoded = json.loads(data.decode())
        print("UDP Received",decoded)      #temporary code comment out after confirm controller inputs are recieved
        return decoded   # returns the SAME list the controller provided

