"""
mpu_data.py

This file defines a simple class, MpuData, to store the 6-axis data
(3-axis accelerometer and 3-axis gyroscope) from the MPU6050 sensor.
"""

class MpuData:
    """
    A simple data container class to hold accelerometer and gyroscope
    readings from the MPU6050 sensor.
    """
    def __init__(self, ax, ay, az, gx, gy, gz):
        """
        Initializes the MpuData object with the raw sensor values.

        Args:
            ax (float): Acceleration along the X-axis (in g's)
            ay (float): Acceleration along the Y-axis (in g's)
            az (float): Acceleration along the Z-axis (in g's)
            gx (float): Gyroscope reading along the X-axis (in degrees/sec)
            gy (float): Gyroscope reading along the Y-axis (in degrees/sec)
            gz (float): Gyroscope reading along the Z-axis (in degrees/sec)
        """
        self.ax = ax
        self.ay = ay
        self.az = az
        self.gx = gx
        self.gy = gy
        self.gz = gz

    def __str__(self):
        """
        Returns a string representation of the MpuData object for easy printing.
        """
        return (f"Accel (g): [x: {self.ax:+.4f}, y: {self.ay:+.4f}, z: {self.az:+.4f}] | "
                f"Gyro (dps): [x: {self.gx:+.4f}, y: {self.gy:+.4f}, z: {self.gz:+.4f}]")