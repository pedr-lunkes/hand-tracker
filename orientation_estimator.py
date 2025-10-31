"""
orientation_estimator.py

Applies an EKF to estimate orientation from "sensor" data topic
and publishes the result to the "orientation" data topic.
"""

import time
import math
import numpy as np
from handlers.mpu_data import MpuData
from orientation_mediator import Mediator

class OrientationData:
    """
    A simple data container for the final orientation output.
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
        return (f"Quat [x,y,z,w]: [{self.qx:+.4f}, {self.qy:+.4f}, {self.qz:+.4f}, {self.qw:+.4f}] | "
                f"Var [x,y,z,w]: [{self.var_qx:+.4E}, {self.var_qy:+.4E}, {self.var_qz:+.4E}, {self.var_qw:+.4E}]")

    def get_8d_vector(self):
        return [self.qx, self.qy, self.qz, self.qw,
                self.var_qx, self.var_qy, self.var_qz, self.var_qw]


class EkfEstimator:
    """
    Implements a 7-state Extended Kalman Filter (EKF) for AHRS.
    
    This class is now driven by callbacks from the Mediator.
    It subscribes to the "sensor" topic.
    It publishes to the "orientation" topic.
      
    State vector x = [qw, qx, qy, qz, bx, by, bz]
    """
    
    def __init__(self, mediator: Mediator):
        """
        Initializes the EKF.
        
        :param mediator: The Pub/Sub mediator to publish/subscribe to.
        """
        self.mediator = mediator
        self.mediator.subscribe("sensor", self.handle_sensor_data)
        
        # --- State and Covariance ---
        self.x = np.zeros(7)
        self.x[0] = 1.0
        
        self.P = np.eye(7)
        self.P[4:, 4:] *= 1.0 
        self.P[:4, :4] *= 0.01

        # -- Noise param --
        var_accel = 0.005
        self.R = np.eye(3) * var_accel
        
        self.var_gyro = 0.005
        self.var_bias = 0.0005
        
        self.Q_noise = np.diag([
            self.var_gyro, self.var_gyro, self.var_gyro,
            self.var_bias, self.var_bias, self.var_bias
        ])

        self.last_time = time.time()
        print("EKF Estimator initialized, waiting for sensor data...")
        
    def _normalize_quaternion(self, q):
        norm = np.linalg.norm(q)
        if norm == 0:
            return np.array([1.0, 0.0, 0.0, 0.0])
        return q / norm

    def handle_sensor_data(self, sensor_data: MpuData):
        """
        This is the main callback.
        Performs one EKF prediction and update step.
        """
        if not sensor_data:
            return

        current_time = time.time()
        dt = current_time - self.last_time
        if dt <= 0.001:
            return # Avoid division by zero
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
            return
        z = accel / accel_norm # This is our measurement vector 'z'

        # --- EKF Prediction Step ---
        q = self.x[:4]; b = self.x[4:]
        w = gyro - b; wx, wy, wz = w
        
        Omega = np.array([
            [0, -wx, -wy, -wz], [wx, 0, wz, -wy],
            [wy, -wz, 0, wx], [wz, wy, -wx, 0]
        ])
        q_dot = 0.5 * Omega @ q
        q_new = q + q_dot * dt
        
        self.x[:4] = self._normalize_quaternion(q_new)
        
        F = np.eye(7)
        F[:4, :4] += 0.5 * Omega * dt
        
        qw, qx, qy, qz = q
        Xi = 0.5 * np.array([
            [-qx, -qy, -qz], [ qw, -qz,  qy],
            [ qz,  qw, -qx], [-qy,  qx,  qw]
        ])
        F[:4, 4:] = -Xi * dt
        
        G = np.zeros((7, 6))
        G[:4, :3] = -Xi * dt
        G[4:, 3:] = np.eye(3) * dt
        
        Q = G @ self.Q_noise @ G.T
        self.P = F @ self.P @ F.T + Q

        # --- EKF Update Step ---
        qw, qx, qy, qz = self.x[:4]
        h = np.array([
            2 * (qx * qz - qw * qy),
            2 * (qy * qz + qw * qx),
            qw * qw - qx * qx - qy * qy + qz * qz
        ])
        
        H = np.zeros((3, 7))
        H[0, 0] = -2 * qy; H[0, 1] = 2 * qz;  H[0, 2] = -2 * qw; H[0, 3] = 2 * qx
        H[1, 0] = 2 * qx;  H[1, 1] = 2 * qw;  H[1, 2] = 2 * qz;  H[1, 3] = 2 * qy
        H[2, 0] = 2 * qw;  H[2, 1] = -2 * qx; H[2, 2] = -2 * qy; H[2, 3] = 2 * qz
        
        y = z - h
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        
        I_KH = np.eye(7) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ self.R @ K.T
        
        self.x[:4] = self._normalize_quaternion(self.x[:4])
        
        q = self.x[:4]
        variances = np.diag(self.P)
        
        orientation_output = OrientationData(
            q[1], q[2], q[3], q[0],
            variances[1], variances[2], variances[3], variances[0]
        )
        
        # Publish the result
        self.mediator.publish("orientation", orientation_output)