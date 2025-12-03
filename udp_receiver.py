import socket
import json

UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", UDP_PORT))
sock.setblocking(False)

def get_udp_data():
    try:
        data, addr = sock.recvfrom(1024)
        decoded = json.loads(data.decode())
        print("UDP Received",decoded)      #temporary code comment out after confirm controller inputs are recieved
        return json.loads(data.decode())   # returns the SAME list the controller provided
    except:
        return None
