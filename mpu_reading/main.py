"""
main.py

Main entry point for the orientation tracking application.
"""

import threading
import sys
import os
from mediator import Mediator

# --- Estimators ---
from estimators.ekf_estimator import EkfEstimator
from estimators.madgwick_estimator import MadgwickEstimator
from estimators.madwick6d_estimator import Madgwick6DEstimator
from estimators.kalman_estimator import KalmanEstimator

from data_visulalization.visualizer_3d import CubeVisualizer
from data_visulalization.ekf_plotter import GaussianPlotter
from data_recorder import Recorder

# --- Import the data handlers ---
from handlers.serial_handler import MpuSerialHandler
from handlers.ble_handler import BleDataHandler
from handlers.keyboard_handler import KeyboardHandler


# --- CONFIGURATION ---
# Options: "BLE", "SERIAL", "KEYBOARD"
DATA_SOURCE = "BLE"

# BLE Settings
BLE_DEVICE_NAME = "HandTracker-MPU" 

# Serial Settings
SERIAL_PORT = '/dev/ttyACM0' 
BAUDRATE = 115200
# ---------------------

def main():
    # Creates a mediator that functions as a server for topics and subscribers
    # allows for easier comunication between classes
    mediator = Mediator()
    

    # ---- Handlers ----
    # Initialize the chosen data handler :D -> where will the data come from?

    handler = None
    # There is support for Bluetooth Low Energy, Serial and a Keyboard for simulation
    if DATA_SOURCE == "BLE":
        print(f"Using BLE data source. Connecting to '{BLE_DEVICE_NAME}'...")
        handler = BleDataHandler(BLE_DEVICE_NAME, mediator)
    elif DATA_SOURCE == "SERIAL":
        print(f"Using Serial data source on {SERIAL_PORT}...")
        handler = MpuSerialHandler(SERIAL_PORT, BAUDRATE, mediator)
    elif DATA_SOURCE == "KEYBOARD":
        # The keyboard mode is only meant for simulation (it has perfect outputs, and no noise
        # in the magnetometer). Must use a yaml accordingly D:
        print(f"Using Keyboard Simulation Mode...")
        handler = KeyboardHandler(mediator)
    else:
        print(f"Error: Unknown DATA_SOURCE '{DATA_SOURCE}'")
        sys.exit(1)

    # Start the handler's thread (it will connect and start publishing to "sensor")
    handler.start()


    # ---- Madgwick definition ----
    # Initializes the Madgwick estimator
    # This is where the all the calculations for orientation and position estimation will be made
    # You must create a .yaml with the parameters fit for your sensor, in order to get the best results :D
    # The results are published in the "orientation" topic
    estimator = Madgwick6DEstimator(mediator, config_path="config_real.yaml")


    # ---- Visual feedback ----
    # This is a plotter that show the uncertainty in the orientation calculations
    # Uncomment if you want to see the Kalman Filter in action >:D
    # plotter = GaussianPlotter(mediator)
    # plotter_thread = threading.Thread(target=plotter.run, daemon=True)
    # plotter_thread.start()

    # This is a visual simulation of a cube that mirrors the orientation and position calculated
    visualizer = CubeVisualizer(mediator)

    recorder = Recorder(mediator, topic_name="orientation")
    recorder_thread = threading.Thread(target=recorder.run_console, daemon=True)
    recorder_thread.start()

    print("Iniciando GUI de Coleta...")
    

    print("Starting simulation... (Close plot window to exit)")
    try:
        # The visualizer is ran in the main thread, so if you close it the program shuts off 
        visualizer.run()
    except KeyboardInterrupt:
        print("\nCaught Ctrl+C, stopping...")
    except Exception as e:
        print(f"Plotter closed due to an error: {e}")
    finally:
        # Stop the data handler 
        handler.stop()
        
        # Stops the recorder
        # recorder.stop()
        
        print("Application shut down.")
        os._exit(0)


if __name__ == "__main__":
    main()