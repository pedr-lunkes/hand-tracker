"""
serial_handler.py

Connects to a serial (USB) port and publishes MPU data to the
"sensor" topic on the mediator.
"""

import serial
import time
import threading
from handlers.mpu_data import MpuData
from orientation_mediator import Mediator

class MpuSerialHandler:
    """
    Manages serial port connection and data reception in a thread.
    Publishes MpuData to the mediator.
    """

    def __init__(self, port, baudrate, mediator: Mediator):
        self.port = port
        self.baudrate = baudrate
        self.mediator = mediator
        self.serial_port = None
        self._running = False
        self.thread = None

    def start(self):
        """Opens the serial port and starts the read thread."""
        try:
            self.serial_port = serial.Serial(self.port, self.baudrate, timeout=1.0)
            print(f"Serial port {self.port} opened.")
            time.sleep(2.0) 
            self.serial_port.flushInput()
            
            self._running = True
            self.thread = threading.Thread(target=self._run_loop, daemon=True)
            self.thread.start()
            
        except serial.SerialException as e:
            print(f"Failed to open serial port {self.port}: {e}")
            self.serial_port = None

    def stop(self):
        """Stops the read thread and closes the serial port."""
        print("Stopping Serial handler...")
        self._running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=2.0)
            
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            print("Serial port closed.")
        print("Serial handler stopped.")

    def is_ready(self):
        """Checks if the serial port is open."""
        return self.serial_port is not None and self.serial_port.is_open

    def _run_loop(self):
        """
        Reads data from serial and publishes it.
        This runs in its own thread.
        """
        print("Serial read thread started.")
        while self._running and self.is_ready():
            try:
                line = self.serial_port.readline().decode('utf-8').strip()
                
                if not line:
                    continue # Timeout

                parts = line.split(' ')
                if len(parts) == 6:
                    ax, ay, az, gx, gy, gz = map(float, parts)
                    mpu_data = MpuData(ax, ay, az, gx, gy, gz)
                    # Publish to the mediator
                    self.mediator.publish("sensor", mpu_data)
                
            except serial.SerialException:
                print("Serial port disconnected. Stopping thread.")
                self._running = False
            except Exception as e:
                # print(f"Serial Read Error: {e}, Line: '{line}'")
                pass # Ignore parse errors
        
        print("Serial read thread finished.")
