"""
main.py

Main entry point for the orientation tracking application.
"""

import threading
import time
import sys
from orientation_mediator import Mediator
from orientation_estimator import EkfEstimator
from data_visulalization.visualizer_3d import CubeVisualizer
from data_visulalization.ekf_plotter import GaussianPlotter

# --- Import the data handlers ---
from handlers.serial_handler import MpuSerialHandler
from handlers.ble_handler import BleDataHandler


# --- CONFIGURATION ---
# Set your desired data source: "BLE" or "SERIAL"
DATA_SOURCE = "BLE"

# BLE Settings
BLE_DEVICE_NAME = "HandTracker-MPU" 

# Serial Settings
SERIAL_PORT = '/dev/ttyACM0' 
BAUDRATE = 115200
# ---------------------

def main():
    print("Starting orientation application...")
    
    # 1. Create the central mediator
    mediator = Mediator()
    
    handler = None
    
    # 2. Initialize the chosen data handler (and pass it the mediator)
    if DATA_SOURCE == "BLE":
        print(f"Using BLE data source. Connecting to '{BLE_DEVICE_NAME}'...")
        handler = BleDataHandler(BLE_DEVICE_NAME, mediator)
    elif DATA_SOURCE == "SERIAL":
        print(f"Using Serial data source on {SERIAL_PORT}...")
        handler = MpuSerialHandler(SERIAL_PORT, BAUDRATE, mediator)
    else:
        print(f"Error: Unknown DATA_SOURCE '{DATA_SOURCE}'")
        sys.exit(1)

    # 3. Start the handler's thread (it will connect and start publishing to "sensor")
    handler.start()

    # 4. Initialize the EKF estimator
    #    It subscribes to "sensor" and publishes to "orientation"
    estimator = EkfEstimator(mediator)

    # 5. Initialize subscribers
    # This plotter subscribes to "orientation"
    plotter = GaussianPlotter(mediator)

    # 6b. Initialize the original Orientation-Only Visualizer
    #     It subscribes to "orientation"
    visualizer = CubeVisualizer(mediator)
    
    print("Starting 3D orientation visualizer thread...")
    visualizer_thread = threading.Thread(target=visualizer.run, daemon=True)

    # 8. Start the chosen visualizer thread
    visualizer_thread.start()

    # 9. Run the plotter in the main thread
    #    This blocks the main thread until the plot window is closed.
    print("Starting real-time plotter... (Close plot window to exit)")
    try:
        plotter.run()
    except KeyboardInterrupt:
        print("\nCaught Ctrl+C, stopping...")
    except Exception as e:
        print(f"Plotter closed due to an error: {e}")
    finally:
        # 10. Clean up
        print("Plotter window closed, stopping all threads...")
        
        # Stop the data handler (disconnects BLE/Serial and stops its thread)
        handler.stop()
        
        # Visualizer thread is daemon, will exit.
        print("Application shut down.")


if __name__ == "__main__":
    main()