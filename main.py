"""
main.py

Main entry point for the orientation tracking application.

This script does the following:
1. Sets the desired data source (BLE or SERIAL).
2. Initializes the chosen data handler.
3. Creates the central Pub/Sub mediator.
4. Injects the handler and mediator into the EKF estimator.
5. Initializes subscribers (Visualizer, Plotter).
6. Starts all threads and manages application lifecycle.
"""

import threading
import time
import sys
from orientation_mediator import OrientationMediator
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
    
    handler = None
    
    # 1. Initialize the chosen data handler
    if DATA_SOURCE == "BLE":
        print(f"Using BLE data source. Connecting to '{BLE_DEVICE_NAME}'...")
        handler = BleDataHandler(BLE_DEVICE_NAME)
    elif DATA_SOURCE == "SERIAL":
        print(f"Using Serial data source on {SERIAL_PORT}...")
        handler = MpuSerialHandler(SERIAL_PORT, BAUDRATE)
    else:
        print(f"Error: Unknown DATA_SOURCE '{DATA_SOURCE}'")
        sys.exit(1)

    # 2. Start the handler (it will connect in its own thread/way)
    handler.start()

    # 3. Create the mediator
    mediator = OrientationMediator()

    # 4. Initialize the publisher (estimator)
    #    Inject the handler and mediator
    estimator = EkfEstimator(handler, mediator)

    # 5. Initialize the subscribers
    visualizer = CubeVisualizer(mediator)
    plotter = GaussianPlotter(mediator)

    # 6. Start the estimator in a background thread
    print("Starting EKF estimator thread...")
    estimator_thread = threading.Thread(target=estimator.run, daemon=True)
    estimator_thread.start()

    # 7. Start the 3D visualizer in a background thread
    print("Starting 3D visualizer thread...")
    visualizer_thread = threading.Thread(target=visualizer.run, daemon=True)
    visualizer_thread.start()

    # 8. Run the plotter in the main thread
    print("Starting real-time plotter... (Close plot window to exit)")
    try:
        plotter.run()
    except KeyboardInterrupt:
        print("\nCaught Ctrl+C, stopping...")
    except Exception as e:
        print(f"Plotter closed due to an error: {e}")
    finally:
        # 9. Clean up
        print("Plotter window closed, stopping all threads...")
        
        # Stop the estimator loop
        estimator.stop()
        
        # Stop the data handler (disconnects BLE/Serial)
        handler.stop()
        
        # Wait for the thread to finish cleaning up
        estimator_thread.join(timeout=2.0)
        
        print("Application shut down.")


if __name__ == "__main__":
    main()

