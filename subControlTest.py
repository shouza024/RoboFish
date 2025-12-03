from udp_receiver import get_udp_data

while True:
    data = get_udp_data()
    print("Control Data",data)