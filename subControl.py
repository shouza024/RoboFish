#!/usr/bin/env python3

import RPi.GPIO as GPIO
from pymavlink import mavutil
from controller import XboxController
from autoFunctions import ConstantDepthControl
from components import Thrusters, TailServo, LED
import time

## Pi Board Setup ----------------------------------------------------------------------------------------
GPIO.setmode(GPIO.BCM)

# Pin Definitions
LED_PIN = 18  # Broadcom pin 18
GPIO.setup(LED_PIN, GPIO.OUT)

## MAVLink Setup -----------------------------------------------------------------------------------------
master = mavutil.mavlink_connection('/dev/serial0', baud=115200)
master.wait_heartbeat()
print(f"Connected to system {master.target_system}, component {master.target_component}")

# Arm the vehicle
def arm_vehicle():
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    time.sleep(1)


## Global Variables --------------------------------------------------------------------------------------
auto_depth_mode = False
prev_depth_state = False
led_on = False
prev_led_state = False


## Main Control Loop -------------------------------------------------------------------------------------
try:
    data = get_udp_data()
    if data is None:
        continue
    arm_vehicle()
    thrusters = Thrusters(master)
    tail_servo = TailServo(master, servo_channel=7)
    led_controller = LED(LED_PIN)
    depth_controller = ConstantDepthControl(thrusters)
    print("Submarine armed. Use Xbox controller to move. Press Ctrl+C to quit.")


    while True:
        data = get_udp_data()
        if data is None:
            continue

        # Map joystick inputs to control variables
        tail_input = data['leftVertical'] > 0.1
        back = data['leftVertical'] < -0.1
        left_right = data['leftHorizontal']
        turn = data['rightHorizontal']
        dive = data['triggerR']
        depthControl = data['y']
        led = data['a']

        # Control thrusters
        thrusters.set_thrusters(forward=back, vertical=dive, yaw=turn)

        # Control tail servo with forward left joystick input
        tail_servo.set_tail_servo_input(tail_input)

        # Control LED brightness with A button
        led_controller.update(led)

        # Constant Depth Control Toggle with 'Y' button
        if depthControl and not prev_depth_state:
            if not depth_controller.enabled:
                depth_controller.enable()   # sets target depth to current distance
            else:
                depth_controller.disable()
        prev_depth_state = depthControl

        # Maintain constant depth if enabled
        depth_controller.maintain_depth()


        time.sleep(0.05)

except KeyboardInterrupt:
    print("Exiting...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    led_controller.cleanup()
