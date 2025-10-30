"""
orientation_estimator.py

This script reads data from an injected data handler (Serial, BLE, etc.)
and applies an Extended Kalman Filter (EKF) to estimate orientation.

This method operates directly on quaternions and is immune to gimbal lock.
"""

import time
import math
import numpy as np
from handlers.mpu_data import MpuData
from orientation_mediator import OrientationMediator

class OrientationData:
    """
    A simple data container for the final orientation output.
    
    Contains the 4 quaternion components (qx, qy, qz, qw) anddata_visulalization.
    their corresponding variances from the EKF's covariance matrix.
    """
    def __init__(self, qx, qy, qz, qw, var_qx, var_qy, var_qz, var_qw):
        self.qx = qx
        self.qy = qy
        self.qz = qz
        self.qw = qw
        self.var_qx = var_qx
        self.var_qy = var_qy
        self.var_qz = var_qz
        self.var_qw = var_qw

    def __str__(self):
        """Returns a string representation for easy printing."""
        return (f"Quat [x,y,z,w]: [{self.qx:+.4f}, {self.qy:+.4f}, {self.qz:+.4f}, {self.qw:+.4f}] | "
                f"Var [x,y,z,w]: [{self.var_qx:+.4E}, {self.var_qy:+.4E}, {self.var_qz:+.4E}, {self.var_qw:+.4E}]")

    def get_8d_vector(self):
        """Returns the 8-dimension vector as requested."""
        return [self.qx, self.qy, self.qz, self.qw,
                self.var_qx, self.var_qy, self.var_qz, self.var_qw]


class EkfEstimator:
    """
    Implements a 7-state Extended Kalman Filter (EKF) for AHRS.
    
    This class is now agnostic to the data source. It just needs
    a 'handler' object with start(), stop(), is_ready(), 
    and read_data() methods.
    
    State vector x = [qw, qx, qy, qz, bx, by, bz]
      - qw, qx, qy, qz: Quaternion components
      - bx, by, bz: Gyroscope bias
      
    Measurement z = [ax, ay, az]
      - ax, ay, az: Normalized accelerometer readings
    """
    
    def __init__(self, data_handler, mediator: OrientationMediator):
        """
        Initializes the EKF.
        
        :param data_handler: An object (like MpuSerialHandler or BleDataHandler)
                             that provides the sensor data.
        :param mediator: The Pub/Sub mediator to publish results to.
        """
        self.handler = data_handler
        self.mediator = mediator
        self._running = False
        
        # --- State and Covariance ---
        # Initial state: [1, 0, 0, 0, 0, 0, 0] (identity quaternion, zero bias)
        self.x = np.zeros(7)
        self.x[0] = 1.0
        
        # Initial state covariance: large uncertainty for bias, small for quaternion
        self.P = np.eye(7)
        self.P[4:, 4:] *= 1.0  # High initial bias uncertainty
        self.P[:4, :4] *= 0.01 # Low initial quaternion uncertainty

        # --- Noise Parameters (TUNABLE) ---
        
        # Measurement noise (accelerometer). Smaller = trust accel more.
        var_accel = 0.01
        self.R = np.eye(3) * var_accel
        
        # Process noise (gyro noise and bias drift).
        # Gyro noise variance. Smaller = trust gyro more.
        self.var_gyro = 0.005
        # Gyro bias random walk variance. Smaller = bias is more stable.
        self.var_bias = 0.0005
        
        # Create the 6x6 discrete process noise covariance matrix
        self.Q_noise = np.diag([
            self.var_gyro, self.var_gyro, self.var_gyro,
            self.var_bias, self.var_bias, self.var_bias
        ])

        self.last_time = time.time()
        
    def is_ready(self):
        """Checks if the data handler is connected and ready."""
        return self.handler is not None and self.handler.is_ready()

    def stop(self):
        """Signals the estimator's run loop to stop."""
        print("Stopping estimator...")
        self._running = False
        # The handler is stopped by main.py, which created it

    def run(self):
        """
        The main loop for the estimator thread.
        Continuously reads data, updates the EKF, and publishes.
        """
        print("Estimator thread started.")
        self._running = True
        
        while not self.is_ready():
            if not self._running:
                print("Estimator stopped before handler was ready.")
                return
            print("Estimator waiting for data handler to be ready...")
            time.sleep(1.0)
            
        print("Estimator is ready, starting EKF loop.")
        self.last_time = time.time() # Reset timer once ready
        
        while self._running:
            orientation = self.update()
            
            if orientation:
                self.mediator.publish(orientation)
            else:
                # Sleep briefly to prevent busy-looping if read_data()
                # is non-blocking and returns None
                time.sleep(0.001)
        
        print("Estimator thread finished.")

    def _normalize_quaternion(self, q):
        norm = np.linalg.norm(q)
        if norm == 0:
            return np.array([1.0, 0.0, 0.0, 0.0])
        return q / norm

    def update(self):
        """
        Reads sensor data and performs one EKF prediction and update step.
        """
        # Note: self.is_ready() is checked in the run() loop
            
        # This is now a non-blocking or short-timeout call
        sensor_data = self.handler.read_data()
        if not sensor_data:
            return None # No new data

        current_time = time.time()
        dt = current_time - self.last_time
        if dt <= 0.001:
            return None
        self.last_time = current_time

        # --- Get Sensor Data ---
        accel = np.array([sensor_data.ax, sensor_data.ay, sensor_data.az])
        gyro = np.array([
            math.radians(sensor_data.gx),
            math.radians(sensor_data.gy),
            math.radians(sensor_data.gz)
        ])
        
        # Normalize accelerometer measurement
        accel_norm = np.linalg.norm(accel)
        if accel_norm == 0:
            return None
        z = accel / accel_norm # This is our measurement vector 'z'

        # --- EKF Prediction Step ---
        
        q = self.x[:4]
        b = self.x[4:]
        
        # Corrected angular velocity
        w = gyro - b
        wx, wy, wz = w
        
        # 1. Predict new state
        # Quaternion kinematics
        Omega = np.array([
            [0, -wx, -wy, -wz],
            [wx, 0, wz, -wy],
            [wy, -wz, 0, wx],
            [wz, wy, -wx, 0]
        ])
        q_dot = 0.5 * Omega @ q
        q_new = q + q_dot * dt
        
        # Bias is random walk (b_new = b)
        
        self.x[:4] = self._normalize_quaternion(q_new)
        # self.x[4:] remains unchanged
        
        # 2. Calculate State Transition Jacobian (F)
        F = np.eye(7)
        # d(q_new)/d(q) = I + 0.5 * Omega(w) * dt
        F[:4, :4] += 0.5 * Omega * dt
        
        # d(q_new)/d(b) = -0.5 * Xi(q) * dt
        qw, qx, qy, qz = q
        Xi = 0.5 * np.array([
            [-qx, -qy, -qz],
            [ qw, -qz,  qy],
            [ qz,  qw, -qx],
            [-qy,  qx,  qw]
        ])
        F[:4, 4:] = -Xi * dt
        
        # 3. Calculate Process Noise Covariance (Q)
        # G maps 6D noise vector [w_gyro, w_bias] to 7D state
        G = np.zeros((7, 6))
        G[:4, :3] = -Xi * dt # Gyro noise to quaternion
        G[4:, 3:] = np.eye(3) * dt # Bias noise to bias
        
        # Q = G * Q_noise * G.T
        Q = G @ self.Q_noise @ G.T
        
        # 4. Predict new covariance
        self.P = F @ self.P @ F.T + Q

        # --- EKF Update Step ---
        
        # 1. Calculate Predicted Measurement h(x)
        qw, qx, qy, qz = self.x[:4]
        h = np.array([
            2 * (qx * qz - qw * qy),
            2 * (qy * qz + qw * qx),
            qw * qw - qx * qx - qy * qy + qz * qz
        ])
        
        # 2. Calculate Measurement Jacobian (H)
        H = np.zeros((3, 7))
        # d(h)/d(q)
        H[0, 0] = -2 * qy
        H[0, 1] = 2 * qz
        H[0, 2] = -2 * qw
        H[0, 3] = 2 * qx
        
        H[1, 0] = 2 * qx
        H[1, 1] = 2 * qw
        H[1, 2] = 2 * qz
        H[1, 3] = 2 * qy
        
        H[2, 0] = 2 * qw
        H[2, 1] = -2 * qx
        H[2, 2] = -2 * qy
        H[2, 3] = 2 * qz
        # d(h)/d(b) is all zeros (which H is initialized to)
        
        # 3. Calculate Innovation
        y = z - h
        
        # 4. Calculate Innovation Covariance
        S = H @ self.P @ H.T + self.R
        
        # 5. Calculate Kalman Gain
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # 6. Update State
        self.x = self.x + K @ y
        
        # 7. Update Covariance (Joseph form for numerical stability)
        I_KH = np.eye(7) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ self.R @ K.T
        
        # 8. Normalize quaternion in state and return
        self.x[:4] = self._normalize_quaternion(self.x[:4])
        
        q = self.x[:4]
        variances = np.diag(self.P)
        
        # Return in [x,y,z,w] and [var_x, var_y, var_z, var_w] order
        return OrientationData(q[1], q[2], q[3], q[0],
                               variances[1], variances[2], variances[3], variances[0])

