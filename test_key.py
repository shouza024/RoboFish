from pymavlink import mavutil
from pynput import keyboard
import time
import threading

# -----------------------------
# PIXHAWK CONNECTION
# -----------------------------
print("Connecting to Pixhawk...")
connection = mavutil.mavlink_connection('COM7', baud=115200)
connection.wait_heartbeat()
print("Heartbeat received!")

# -----------------------------
# ARM VEHICLE
# -----------------------------
print("Arming Pixhawk...")
connection.mav.command_long_send(
    connection.target_system,
    connection.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0
)

while True:
    hb = connection.recv_match(type='HEARTBEAT', blocking=True)
    if hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
        print("ARMED!")
        break
    time.sleep(0.1)


# -----------------------------
# CHANNEL ASSIGNMENTS
# -----------------------------
motor_channels = [1, 2, 3, 4]   # Main outputs 1–4
servo_channel = 9               # AUX1 = SERVO9  (IMPORTANT!)

# Motor PWM definitions
PWM_STOP = 1500
PWM_HALF = 1750
PWM_FULL = 1900

# Servo oscillation limits (±65 degrees -> roughly ±360 µs)
SERVO_CENTER =1500
SERVO_RANGE = 500    
SERVO_MIN = SERVO_CENTER - SERVO_RANGE
SERVO_MAX = SERVO_CENTER + SERVO_RANGE

# System state
servo_running = False
stop_script = False
motor_pwms = [PWM_STOP] * 4
servo_pwm = SERVO_CENTER


# -----------------------------
# SEND MOTOR PWM (RC override)
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
# SEND SERVO PWM via DO_SET_SERVO
# -----------------------------
def set_servo_pwm(channel, pwm):
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
# SERVO OSCILLATION THREAD
# -----------------------------
def servo_oscillator():
    global servo_pwm, servo_running

    pwm = SERVO_CENTER
    direction = 1

    while servo_running:
        set_servo_pwm(servo_channel, pwm)
        servo_pwm = pwm

        pwm += direction * 40   # smooth step

        if pwm >= SERVO_MAX:
            pwm = SERVO_MAX
            direction = -1
        elif pwm <= SERVO_MIN:
            pwm = SERVO_MIN
            direction = 1

        time.sleep(0.03)# speed of oscillation


# -----------------------------
# KEYBOARD HANDLER
# -----------------------------
def on_press(key):
    global motor_pwms, servo_running, stop_script,SERVO_RANGE

    try:
        k = key.char.lower()
    except:
        k = None

    # Throttle commands
    if k == 'f':
        motor_pwms = [PWM_FULL, PWM_FULL,
                      PWM_FULL, PWM_FULL]
        send_motor_pwm()
        print("Full throttle")

    elif k == 'h':
        motor_pwms = [PWM_HALF, PWM_HALF,
                      PWM_HALF, PWM_HALF]
        send_motor_pwm()
        print("Half throttle")

    elif k == 'z':
        motor_pwms = [PWM_STOP]*4
        send_motor_pwm()
        print("Motors stopped")

    # Individual motor increase
    elif k in ['1', '2', '3', '4']:
        idx = int(k) - 1
        motor_pwms[idx] += 50
        send_motor_pwm()
        print(f"Motor {idx+1} PWM: {motor_pwms[idx]}")

    # Individual motor decrease
    elif k in ['q', 'w', 'e', 'r']:
        idx = ['q','w','e','r'].index(k)
        motor_pwms[idx] -= 50
        send_motor_pwm()
        print(f"Motor {idx+1} PWM: {motor_pwms[idx]}")

    # Servo oscillation start
    elif k == 's':
        if not servo_running:
            servo_running = True
            threading.Thread(target=servo_oscillator, daemon=True).start()
            print("Servo oscillation started")
    elif k == 'd':
        if not servo_running:
            servo_running = True
            SERVO_RANGE=+50
            threading.Thread(target=servo_oscillator, daemon=True).start()
            print("Servo oscillation started")        

    # Servo oscillation stop
    elif k == 'x':
        servo_running = False
        set_servo_pwm(servo_channel, SERVO_CENTER)
        print("Servo oscillation stopped")

    # ESC to stop script
    elif key == keyboard.Key.esc:
        stop_script = True
        return False


# Start listening
listener = keyboard.Listener(on_press=on_press)
listener.start()


# -----------------------------
# MAIN LOOP
# -----------------------------
print("Running control loop. Press ESC to quit.")
while not stop_script:
    send_motor_pwm()
    time.sleep(0.1)

# -----------------------------
# DISARMxsxsxsxsxsxsxsxsxsx1sx111qqqq1111111qqqqqqqs
# -----------------------------
connection.mav.command_long_send(
    connection.target_system,
    connection.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    0, 0, 0, 0, 0, 0, 0
)

print("Disarmed and exiting.")
