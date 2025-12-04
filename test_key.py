from pymavlink import mavutil
from pynput import keyboard
import time
import threading

# -----------------------------
# Connect to Pixhawk
# -----------------------------
print("Connecting to Pixhawk...")
connection = mavutil.mavlink_connection('COM7', baud=115200)
connection.wait_heartbeat()
print("Heartbeat received!")

# -----------------------------
# Arm the vehicle
# -----------------------------
print("Arming Pixhawk...")
connection.mav.command_long_send(
    connection.target_system,
    connection.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0
)

# Wait until armed
while True:
    hb = connection.recv_match(type='HEARTBEAT', blocking=True)
    if hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
        print("ARMED!")
        break
    time.sleep(0.1)

# -----------------------------
# Channel definitions
# -----------------------------
motor_channels = [1, 2, 3, 4]  # ESCs on MAIN OUT 1–4
servo_channel = 7               # Servo on AUX7

# PWM values
PWM_STOP = 1500
PWM_HALF = 1750
PWM_FULL = 1900

SERVO_MIN = 1100
SERVO_MAX = 1900

# States
servo_running = False
stop_script = False

# Motor PWM storage
motor_pwms = [PWM_STOP] * 4
servo_pwm = 1500  # persistent servo PWM

# -----------------------------
# Send motor PWM via RC override
# -----------------------------
def send_motor_pwm():
    rc = [0] * 16
    for i, ch in enumerate(motor_channels):
        rc[ch - 1] = motor_pwms[i]
    connection.mav.rc_channels_override_send(
        connection.target_system,
        connection.target_component,
        *rc
    )

# -----------------------------
# Send servo PWM via MAV_CMD_DO_SET_SERVO
# -----------------------------
def set_servo_pwm(channel, pwm):
    """
    channel: 1–14 (matches SERVOx_FUNCTION)
    pwm: 1000–2000 µs
    """
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        channel,
        pwm,
        0, 0, 0, 0, 0
    )

# -----------------------------
# Servo oscillation thread
# -----------------------------
def servo_oscillator():
    global servo_running, servo_pwm
    pwm = servo_pwm
    direction = 1
    while servo_running:
        servo_pwm = pwm
        set_servo_pwm(servo_channel, pwm)  # send independent of motors

        pwm += direction * 20
        if pwm >= SERVO_MAX:
            pwm = SERVO_MAX
            direction = -1
        elif pwm <= SERVO_MIN:
            pwm = SERVO_MIN
            direction = 1

        time.sleep(0.05)

# -----------------------------
# Keyboard handler
# -----------------------------
def on_press(key):
    global motor_pwms, servo_running, stop_script, servo_pwm

    try:
        k = key.char.lower()
    except:
        k = None

    # Throttle control (1&2 forward, 3&4 reverse if bidirectional ESCs)
    if k == 'f':
        motor_pwms[0] = PWM_FULL
        motor_pwms[1] = PWM_FULL
        motor_pwms[2] = PWM_STOP - (PWM_FULL - PWM_STOP)
        motor_pwms[3] = PWM_STOP - (PWM_FULL - PWM_STOP)
        send_motor_pwm()
        print("Full throttle applied")
    elif k == 'h':
        motor_pwms[0] = PWM_HALF
        motor_pwms[1] = PWM_HALF
        motor_pwms[2] = PWM_STOP - (PWM_HALF - PWM_STOP)
        motor_pwms[3] = PWM_STOP - (PWM_HALF - PWM_STOP)
        send_motor_pwm()
        print("Half throttle applied")
    elif k == 'z':
        motor_pwms = [PWM_STOP]*4
        send_motor_pwm()
        print("Motors stopped")

    # Individual motor adjustments (increase)
    elif k == '1':
        motor_pwms[0] += 50
        send_motor_pwm()
        print(f"Motor 1 PWM: {motor_pwms[0]}")
    elif k == '2':
        motor_pwms[1] += 50
        send_motor_pwm()
        print(f"Motor 2 PWM: {motor_pwms[1]}")
    elif k == '3':
        motor_pwms[2] += 50
        send_motor_pwm()
        print(f"Motor 3 PWM: {motor_pwms[2]}")
    elif k == '4':
        motor_pwms[3] += 50
        send_motor_pwm()
        print(f"Motor 4 PWM: {motor_pwms[3]}")

    # Individual motor adjustments (decrease)
    elif k == 'q':
        motor_pwms[0] -= 50
        send_motor_pwm()
        print(f"Motor 1 PWM: {motor_pwms[0]}")
    elif k == 'w':
        motor_pwms[1] -= 50
        send_motor_pwm()
        print(f"Motor 2 PWM: {motor_pwms[1]}")
    elif k == 'e':
        motor_pwms[2] -= 50
        send_motor_pwm()
        print(f"Motor 3 PWM: {motor_pwms[2]}")
    elif k == 'r':
        motor_pwms[3] -= 50
        send_motor_pwm()
        print(f"Motor 4 PWM: {motor_pwms[3]}")

    # Servo oscillation
    elif k == 's':
        if not servo_running:
            servo_running = True
            threading.Thread(target=servo_oscillator, daemon=True).start()
            print("Starting servo oscillation")
    elif k == 'x':
        servo_running = False
        set_servo_pwm(servo_channel, 1500)  # return to center
        print("Stopping servo oscillation")

    # Stop script
    elif key == keyboard.Key.esc:
        stop_script = True
        return False

# -----------------------------
# Start keyboard listener
# -----------------------------
listener = keyboard.Listener(on_press=on_press)
listener.start()

# -----------------------------
# Main loop
# -----------------------------
while not stop_script:
    send_motor_pwm()  # continuously send motor PWM
    time.sleep(0.1)

# -----------------------------
# Disarm vehicle
# -----------------------------
connection.mav.command_long_send(
    connection.target_system,
    connection.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    0, 0, 0, 0, 0, 0, 0
)

print("Disarmed. Exiting.")
