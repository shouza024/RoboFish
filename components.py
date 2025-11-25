import RPi.GPIO as GPIO
import time
from picamera2 import Picamera2
from pymavlink import mavutil
import cv2

# Set up GPIO mode
GPIO.setmode(GPIO.BCM)

class FrontCamera:
    def __init__(self, resolution=(1920, 1080), framerate=30, display=False):
        self.picam2 = Picamera2()
        self.display = display
        
        config = self.picam2.create_video_configuration(
            main={"size": resolution, "format": "RGB888"}
        )
        self.picam2.configure(config)
        self.picam2.start()
        time.sleep(0.5)  # allow sensor to warm up

    def capture_frame(self):
        frame = self.picam2.capture_array()
        return frame
    
    # Shows live window of camera feed
    def show_stream(self):
        if not self.display:
            raise RuntimeError("Display mode disabled. Enable display=True to use.")
        
        while True:
            frame = self.get_frame()
            cv2.imshow("Front Camera", frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        cv2.destroyAllWindows()

    def stop(self):
        self.picam2.close()


class Thrusters:
    def __init__(self, mav_master):
        self.master = mav_master

    def set_thrusters(self, forward=0.0, yaw=0.0, vertical=0.0):
        """
        Thruster mixing for an inverted quad (4 upward-facing thrusters).
        forward:  forward/backward motion  (-1 to 1)
        yaw:      turn left/right           (-1 to 1)
        vertical: up/down motion            (-1 to 1)
                  positive = dive (down), negative = ascend (up)
        """

        # Motor mixing (simplified for X configuration)
        # Front-left (T1), Front-right (T2), Rear-left (T3), Rear-right (T4)
        t1 = vertical - forward + yaw   # Front-left
        t2 = vertical - forward - yaw   # Front-right
        t3 = vertical + forward + yaw   # Rear-left
        t4 = vertical + forward - yaw   # Rear-right

        # Clamp motor outputs to [-1, 1]
        t1 = max(-1, min(1, t1))
        t2 = max(-1, min(1, t2))
        t3 = max(-1, min(1, t3))
        t4 = max(-1, min(1, t4))

        # Convert to MAVLink range [-1000, 1000]
        # (ArduSub typically expects 1500 neutral, 1900 high, 1100 low → scaled here)
        def scale(x): return int(x * 1000)

        # Send to PX4/ArduSub using manual_control
        # Here we just send aggregate commands (full mixer would be per-channel)
        self.master.mav.manual_control_send(
            self.master.target_system,
            int(forward * 1000),       # X: forward/back
            0,                         # Y: strafe
            int(500 + vertical * 500), # Z: thrust (inverted)
            int(yaw * 1000),           # R: yaw
            0
        )

        # For debugging / simulation logs
        print(f"[THRUSTERS] FL={t1:.2f}, FR={t2:.2f}, RL={t3:.2f}, RR={t4:.2f} | F={forward:.2f} Y={yaw:.2f} V={vertical:.2f}")


class TailServo:
    def __init__(self, mav_master, servo_channel=7):
        self.master = mav_master
        self.servo_channel = servo_channel

    def set_tail_servo_pwm(self, pwm_value):
        """Direct PWM control (1000–2000)."""
        pwm_value = max(1000, min(2000, pwm_value))
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,
            self.servo_channel,
            pwm_value,
            0, 0, 0, 0, 0
        )
        print(f"[TAIL SERVO] PWM={pwm_value} on channel {self.servo_channel}")

    def set_tail_servo_input(self, input_value):
        """Accepts normalized input (-1 to 1)."""
        input_value = max(-1, min(1, input_value))
        pwm_value = 1500 + (input_value * 500)  # center 1500, ±500
        self.set_tail_servo_pwm(pwm_value)


class LED:
    def __init__(self, led_pin):
        """Initialize LED controller."""
        self.led_pin = led_pin
        self.led_on = False
        self.prev_a_state = False

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.led_pin, GPIO.OUT)
        GPIO.output(self.led_pin, GPIO.LOW)

    def update(self, a_button_state):
        """
        Call this every loop with the current A button state (True/False).
        Toggles the LED on button press.
        """
        if a_button_state and not self.prev_a_state:
            # Toggle LED state
            self.led_on = not self.led_on
            GPIO.output(self.led_pin, GPIO.HIGH if self.led_on else GPIO.LOW)
            print("LED light ON" if self.led_on else "LED light OFF")

        # Remember last button state
        self.prev_a_state = a_button_state

    def cleanup(self):
        """Turn off LED and clean up GPIO on exit."""
        GPIO.output(self.led_pin, GPIO.LOW)
        GPIO.cleanup()
        print("LED controller cleaned up.")