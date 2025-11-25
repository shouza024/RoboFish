# Constant Depth

from pymavlink import mavutil
from components import Thrusters
import time


class ConstantDepthControl:
    def __init__(self, mav_master, thrusters):
        self.master = mav_master
        self.thrusters = thrusters
        self.enabled = False
        self.target_accel_z = None  # IMU target Z-accel baseline

    def read_vertical_accel(self):
        """Reads Z-axis acceleration from RAW_IMU."""
        msg = self.master.recv_match(type='RAW_IMU', blocking=False)
        if msg is None:
            return None
        return msg.zacc  # mg units

    def enable(self):
        """Capture the current acceleration as the target baseline."""
        z = self.read_vertical_accel()
        if z is None:
            print("Gyro data unavailable. Cannot enable depth hold.")
            return

        self.target_accel_z = z
        self.enabled = True
        print(f"[DEPTH HOLD] Enabled. Target Z-accel = {z}")

    def disable(self):
        self.enabled = False
        self.thrusters.set_thrusters(vertical=0)
        print("[DEPTH HOLD] Disabled.")

    def maintain(self):
        """Applies downward thrust only if robot floats upward."""
        if not self.enabled:
            return

        z = self.read_vertical_accel()
        if z is None:
            print("[DEPTH HOLD] IMU read failed.")
            return

        error = self.target_accel_z - z  # positive = too light / rising

        # Simple proportional control
        Kp = 0.0005
        thrust = max(0, min(1.0, error * Kp))  # Only downward thrust!

        self.thrusters.set_thrusters(vertical=thrust)
