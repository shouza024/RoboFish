from udp_receiver import get_udp_data
import time

while True:
    try:
        data = get_udp_data()
        print("Control Data",data)
        time.sleep(100)
    except Exception as e:
        print("Error",e)