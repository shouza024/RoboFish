#!/usr/bin/env python3
"""
Simplified ArduSub-style Python control for Pixhawk 4 + Raspberry Pi 5
System: 4 thrusters, 1 tail servo, 2 cameras, 1 LED light
Reference code only. Main code to run is subControl.py
"""

from pymavlink import mavutil
import time
import cv2

# ======================================
# MAVLink Setup
# ======================================

serial = '/dev/serial0'     # Adjust as necessary
baudrate = 115200           # Adjust as necessary

print("Connecting to Pixhawk...")
master = mavutil.mavlink_connection(serial, baud=baudrate)
master.wait_heartbeat()
print(f"Connected to system {master.target_system}, component {master.target_component}")

# Arm the vehicle
def arm_vehicle():
    print("Arming thrusters...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    time.sleep(2)

# Disarm the vehicle
def disarm_vehicle():
    print("Disarming thrusters...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0
    )

# ======================================
# Motor and Servo Control
# ======================================

def set_thrusters(forward=0, vertical=0, yaw=0):
    """
    Send manual control commands.
    forward, vertical, yaw ∈ [-1000, 1000]
    """
    master.mav.manual_control_send(
        master.target_system,
        forward,  # X axis (surge)
        0,        # Y axis (sway)
        500 - vertical // 2,  # Z axis (heave, reversed scale)
        yaw,      # R axis (yaw)
        0         # Buttons
    )

def set_tail_servo(pwm=1500):
    """
    Control the tail servo using RC channel override.
    pwm: 1100 (left) to 1900 (right)
    """
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        0, 0, 0, 0, 0, 0, 0, pwm
    )

def set_led(brightness_pwm=1800):
    """
    Control LED brightness (via RC channel 9 for example)
    """
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        0, 0, 0, 0, 0, 0, 0, 0,
        brightness_pwm
    )

# ======================================
# Camera Feed (Front and Bottom)
# ======================================

def start_cameras():
    front_cam = cv2.VideoCapture(0)  # front-facing
    bottom_cam = cv2.VideoCapture(1) # bottom-facing

    if not front_cam.isOpened() or not bottom_cam.isOpened():
        print("Error: Could not open cameras.")
        return

    print("Cameras active. Press 'q' to quit.")
    while True:
        ret1, frame1 = front_cam.read()
        ret2, frame2 = bottom_cam.read()

        if ret1:
            cv2.imshow('Front Camera', frame1)
        if ret2:
            cv2.imshow('Bottom Camera', frame2)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    front_cam.release()
    bottom_cam.release()
    cv2.destroyAllWindows()

# ======================================
# Example Run Sequence
# ======================================

try:
    arm_vehicle()
    time.sleep(1)

    print("Starting simple movement sequence...")
    for i in range(3):
        print("Moving forward...")
        set_thrusters(forward=500)
        time.sleep(2)

        print("Turning right...")
        set_thrusters(yaw=500)
        time.sleep(2)

        print("Stopping...")
        set_thrusters(forward=0, yaw=0)
        time.sleep(2)

    print("Adjusting tail servo and LED...")
    set_tail_servo(1600)
    set_led(1900)

    print("Starting camera feeds...")
    start_cameras()

finally:
    disarm_vehicle()
    print("Shutdown complete.")
