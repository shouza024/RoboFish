from udp_receiver import get_udp_data

while True:
    try:
        data = get_udp_data()
        print("Control Data",data)
    except Exception as e:
        print("Error",e)