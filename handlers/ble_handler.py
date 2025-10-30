"""
ble_handler.py

Connects to a BLE device and publishes MPU data to the
"sensor" topic on the mediator.
"""

import asyncio
import threading
import queue
from bleak import BleakScanner, BleakClient
from handlers.mpu_data import MpuData
from orientation_mediator import Mediator

UART_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
UART_TX_CHAR_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

class BleDataHandler:
    """
    Manages BLE connection and publishes data to the mediator.
    """
    
    def __init__(self, device_name, mediator: Mediator):
        self.device_name = device_name
        self.mediator = mediator
        self.client = None
        self.is_running = False
        self.thread = None
        self.loop = None
        self.device_found = threading.Event()

    def _notification_handler(self, sender, data: bytearray):
        """Callback for incoming BLE notifications."""
        try:
            decoded_data = data.decode('utf-8').strip()
            parts = decoded_data.split(',')
            
            if len(parts) == 6:
                ax, ay, az, gx, gy, gz = map(float, parts)
                mpu_data = MpuData(ax, ay, az, gx, gy, gz)
                # Publish to the mediator
                self.mediator.publish("sensor", mpu_data)
        except Exception as e:
            print(f"BLE Parse Error: {e}, Data: {data}")

    async def _run_ble_loop(self):
        """The core asyncio loop for scanning, connecting, and listening."""
        print(f"Starting BLE scan for '{self.device_name}'...")
        device = await BleakScanner.find_device_by_name(self.device_name, timeout=10.0)
        
        if not device:
            print(f"Could not find device '{self.device_name}'.")
            self.device_found.set() 
            return

        print(f"Found device: {device.address}")
        self.device_found.set()

        async with BleakClient(device) as client:
            self.client = client
            print("Connected to device.")
            try:
                await client.start_notify(UART_TX_CHAR_UUID, self._notification_handler)
                print("Subscribed to notifications. Listening for data...")
                while self.is_running:
                    await asyncio.sleep(0.1)
                
                await client.stop_notify(UART_TX_CHAR_UUID)
                print("Unsubscribed from notifications.")
                
            except Exception as e:
                print(f"BLE Error: {e}")
            finally:
                self.client = None
                print("Disconnected.")

    def _run_async_wrapper(self):
        """Sets up and runs the asyncio event loop in this thread."""
        try:
            self.loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.loop)
            self.loop.run_until_complete(self._run_ble_loop())
        except Exception as e:
            print(f"Async loop error: {e}")
        finally:
            self.is_running = False

    def start(self):
        """Starts the BLE connection thread."""
        if self.is_running:
            return
            
        print("Starting BLE handler...")
        self.is_running = True
        self.device_found.clear()
        self.thread = threading.Thread(target=self._run_async_wrapper, daemon=True)
        self.thread.start()
        
        self.device_found.wait() 

    def stop(self):
        """Stops the BLE connection thread."""
        print("Stopping BLE handler...")
        self.is_running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=2.0)
        print("BLE handler stopped.")

    def is_ready(self):
        """Checks if the BLE client is connected."""
        return self.client is not None and self.client.is_connected
