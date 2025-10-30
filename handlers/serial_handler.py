"""
serial_handler.py

This script implements a data handler for reading MPU6050 data
from a standard serial (USB) port.
"""

import serial
import time
from handlers.mpu_data import MpuData

class MpuSerialHandler:
    """
    Manages serial port connection and data reception.
    Provides a blocking `read_data()` method.
    """

    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        self.serial_port = None

    def start(self):
        """Opens the serial port."""
        try:
            self.serial_port = serial.Serial(self.port, self.baudrate, timeout=1.0)
            print(f"Serial port {self.port} opened.")
            # Allow time for connection to establish and Arduino to reset
            time.sleep(2.0) 
            self.serial_port.flushInput()
        except serial.SerialException as e:
            print(f"Failed to open serial port {self.port}: {e}")
            self.serial_port = None

    def stop(self):
        """Closes the serial port."""
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            print("Serial port closed.")

    def is_ready(self):
        """Checks if the serial port is open."""
        return self.serial_port is not None and self.serial_port.is_open

    def read_data(self):
        """
        Reads one line of data from the serial port and parses it.
        This is a blocking call.
        """
        if not self.is_ready():
            return None

        try:
            line = self.serial_port.readline().decode('utf-8').strip()
            
            if not line:
                return None # Timeout

            # This assumes the *original* Arduino code's space-separated format
            # "ax ay az gx gy gz"
            parts = line.split(' ')

            if len(parts) == 6:
                ax, ay, az, gx, gy, gz = map(float, parts)
                return MpuData(ax, ay, az, gx, gy, gz)
            else:
                # print(f"Serial Parse Warning: Unexpected format. Got {len(parts)} parts. Line: '{line}'")
                return None

        except Exception as e:
            print(f"Serial Read Error: {e}, Line: '{line}'")
            return None
