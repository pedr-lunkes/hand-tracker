"""
orientation_estimator.py

Extended Kalman filter for estimation position and orientation (based on a MPU9250)
You can make it work with a MPU6500, just remove the magnetomete parâmeters. This will,
however, consideraly worsen the yaw results.
"""

import time
import math
import yaml
import numpy as np

from handlers.mpu_data import MpuData
from mediator import Mediator


# Defines a class for conveying the calculated data from EKF
class OrientationData:
    def __init__(self, qx, qy, qz, qw, x, y, z, var_qx, var_qy, var_qz, var_qw):
        self.qx, self.qy, self.qz, self.qw = qx, qy, qz, qw
        self.x, self.y, self.z = x, y, z
        self.var_qx, self.var_qy, self.var_qz, self.var_qw = var_qx, var_qy, var_qz, var_qw

class EkfEstimator:
    """Class that makes the estimation and update of the current position + orientation
        through the use of a Extended Kalman Filter, an algorithm that cleans noise and 
        estimates the values.
        With sensor fusion (magnetometer + accelerometer + gyroscope) it is possible to get
        a pretty good relative orientation estimation, with minimal erros to yaw.
    """
    def __init__(self, mediator: Mediator, config_path="ekf_config.yaml"):
        self.mediator = mediator
        
        # --- YAML configurations ---
        print(f"Loading EKF configuration from {config_path}...")
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)
            
        # Topics
        self.input_topic = self.config['topics']['input_topic']
        self.output_topic = self.config['topics']['output_topic']
        self.mediator.subscribe(self.input_topic, self.handle_sensor_data)
        
        # --- EKF State ---
        self.x_state = np.zeros(7)
        self.x_state[0] = 1.0 
        
        # Configuration of P Matrix (initial covariance)
        p_orient = self.config['ekf']['initial_covariance']['orientation_scale']
        p_bias = self.config['ekf']['initial_covariance']['bias_scale']
        self.P = np.eye(7)
        self.P[:4, :4] *= p_orient
        self.P[4:, 4:] *= p_bias

        # --- EKF Noise Parameters ---
        r_accel = self.config['ekf']['measurement_noise']['accel_factor']
        r_mag = self.config['ekf']['measurement_noise']['mag_factor']
        q_noise_list = self.config['ekf']['process_noise']
        
        self.R_accel = np.eye(3) * r_accel
        self.R_mag = np.eye(3) * r_mag
        self.Q_noise = np.diag(q_noise_list)

        # --- Magnetic calibration ---
        self.mag_bias = np.array(self.config['magnetometer']['bias'])
        self.mag_scale = np.array(self.config['magnetometer']['scale'])
        self.mag_ref = None 

        # --- Positional variables (DEAD RECKONING) ---
        # Dead reckoning corresponds to calculatin the position based on the last 
        # known location, with new information of velocity and orientation (where is it headed)
        self.pos = np.array([0.0, 0.0, 0.0]) 
        self.vel = np.array([0.0, 0.0, 0.0]) 
        
        dr_conf = self.config['dead_reckoning']
        self.gravity_vector = np.array(dr_conf['gravity_vector'])
        self.calibration_samples_req = dr_conf['calibration_frames']
        
        # Thresholds and Drag
        self.accel_threshold = dr_conf['thresholds']['general']
        self.accel_threshold_z = dr_conf['thresholds']['z_axis']
        self.velocity_drag = dr_conf['drag']['xy']
        self.velocity_drag_z = dr_conf['drag']['z']
        self.stat_tolerance = dr_conf['stationary_tolerance']
        
        # Flags
        self.use_floor = dr_conf['floor_constraint']
        self.return_center = dr_conf['return_to_center']

        # Internal control variables
        self.calibration_samples = []
        self.g_magnitude = 0.0
        self.is_calibrating_g = True
        self.dt_min = self.config['ekf']['dt_min']

        self.last_time = time.time()
        print("EKF Initialized via YAML. Calibrating Gravity...")

    def _normalize_quaternion(self, q):
        """
        Normalizes the quaternion to unit length. 
        q_norm = q / ||q||.
        This is crucial because rotations are represented by unit quaternions.
        """
        norm = np.linalg.norm(q)
        return q / norm if norm > 0 else np.array([1.0, 0.0, 0.0, 0.0])

    def _quaternion_to_rotation_matrix(self, q):
        """
        Converts a quaternion into a 3x3 Rotation Matrix (Direction Cosine Matrix).
        This matrix R is used to transform vectors from the Body Frame (Sensor) 
        to the World Frame (Navigation). 
        """
        qw, qx, qy, qz = q
        return np.array([
            [1 - 2*qy**2 - 2*qz**2, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
            [2*qx*qy + 2*qz*qw, 1 - 2*qx**2 - 2*qz**2, 2*qy*qz - 2*qx*qw],
            [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx**2 - 2*qy**2]
        ])

    def handle_sensor_data(self, sensor_data: MpuData):
        """
        Main EKF Loop: Prediction -> Correction (Update).
        
        1. Prediction: Estimates the next state based on the system dynamics (Gyroscope).
        2. Correction: Corrects the estimate using measurements (Accel + Mag).
        """
        if not sensor_data: return

        # Calculate time step (dt) for integration
        current_time = time.time()
        dt = current_time - self.last_time
        if dt <= self.dt_min: return
        self.last_time = current_time

        accel_raw = np.array([sensor_data.ax, sensor_data.ay, sensor_data.az])
        

        # --- Gravity calibration (preprocessing) ---
        # Gravity needs to be calibrated so it can be subtracted from the accelerometer data
        # later on, in order to obtain the relative movement (and position tracking)
        if self.is_calibrating_g:
            self.calibration_samples.append(np.linalg.norm(accel_raw))
            if len(self.calibration_samples) > self.calibration_samples_req:
                self.g_magnitude = np.mean(self.calibration_samples)
                self.is_calibrating_g = False
                print(f"Gravity Calibrated: {self.g_magnitude:.2f} units = 1 G")
            return 


        # --- Magnetometer calibration (preprocessing still) ---
        # Magnetometer: Hard Iron / Soft Iron correction
        # v_corrected = Scale * (v_raw - Bias) (applied linear transformations D:)
        mx = (sensor_data.mx - self.mag_bias[0]) * self.mag_scale[0]
        my = (sensor_data.my - self.mag_bias[1]) * self.mag_scale[1]
        mz = (sensor_data.mz - self.mag_bias[2]) * self.mag_scale[2]
        
        # Aligning Sensor Axes with World Axes (Swapping axes to match NED/ENU convention)
        mag_body = np.array([my, mx, -mz]) 
        
        # Normalization of the data
        mag_norm = np.linalg.norm(mag_body)
        z_mag = mag_body / mag_norm if mag_norm > 0 else np.zeros(3)

        accel_norm = np.linalg.norm(accel_raw)
        z_accel = accel_raw / accel_norm if accel_norm > 0 else np.zeros(3)

        # Initialize reference magnetic vector if first run
        # This is important because this frame will be used to correct the cumulative erros in
        # gyroscope readings, and make the yaw prediction
        if self.mag_ref is None: self.mag_ref = z_mag; return


        # -----------------------------------------------------------
        # PREDICTION (Time Update)
        # Physics Model: Quaternion Kinematics q_dot = 1/2 * Omega * q
        # We will use quaternions to avoid the gimbal lock problem :)
        # -----------------------------------------------------------7
        
        # 1 - Convert Gyro (deg/s) to rad/s
        gyro = np.array([math.radians(x) for x in [sensor_data.gx, sensor_data.gy, sensor_data.gz]])

        # 2 - State Vector Partitioning: q (Orientation) and b (Gyro Bias)
        q = self.x_state[:4]; b = self.x_state[4:]

        # 3 - Correct angular velocity with estimated bias
        w = gyro - b

        # 4 - Construct the Omega Matrix (Skew-Symmetric Matrix for Angular Velocity)
        # This matrix represents the differential rotation operator.
        Omega = np.array([[0, -w[0], -w[1], -w[2]], [w[0], 0, w[2], -w[1]], [w[1], -w[2], 0, w[0]], [w[2], w[1], -w[0], 0]])
        
        # 5 - State Integration (Euler Method)
        # x_k = x_{k-1} + (dx/dt) * dt
        q_new = q + 0.5 * Omega @ q * dt
        self.x_state[:4] = self._normalize_quaternion(q_new)
        
        # 6 - Jacobian of the State Transition Function (Matrix F)
        # F = Partial derivatives of the system dynamics w.r.t the state variables.
        # This linearizes the non-linear system around the current operating point.
        F = np.eye(7)
        F[:4, :4] += 0.5 * Omega * dt

        # Partial derivative w.r.t Bias involves this auxiliary matrix Xi
        Xi = 0.5 * np.array([[-q[1], -q[2], -q[3]], [q[0], -q[3], q[2]], [q[3], q[0], -q[1]], [-q[2], q[1], q[0]]])
        F[:4, 4:] = -Xi * dt; 
        
        # 7 Covariance Projection (P_prediction)
        # P_k = F * P_{k-1} * F^T + Q
        # This propagates the uncertainty ellipsoid forward in time.
        G = np.zeros((7, 6)); G[:4, :3] = -Xi * dt; G[4:, 3:] = np.eye(3) * dt
        self.P = F @ self.P @ F.T + (G @ self.Q_noise @ G.T)

        
        # -----------------------------------------------------------
        # FIRST CORRECTION / UPDATE (Measurement Update) - ACCELEROMETER
        # We correct the orientation assuming the accelerometer measures Gravity (Down)
        # -----------------------------------------------------------
        qw, qx, qy, qz = self.x_state[:4]

        # 1 Predicted Measurement (h(x))
        # Where do we expect gravity to be in the sensor frame based on current rotation?
        # Ideally, Gravity in Body Frame = R_inverse * [0, 0, 1]
        h_accel = np.array([2*(qx*qz - qw*qy), 2*(qy*qz + qw*qx), qw**2 - qx**2 - qy**2 + qz**2])

        # 2 Jacobian of Measurement Function (Matrix H)
        # H = Jacobian(h_accel) w.r.t state vector (x).
        # Describes how the gravity vector changes when the quaternion changes.
        H_accel = np.zeros((3, 7))
        H_accel[0,0]=-2*qy; H_accel[0,1]=2*qz; H_accel[0,2]=-2*qw; H_accel[0,3]=2*qx
        H_accel[1,0]=2*qx;  H_accel[1,1]=2*qw; H_accel[1,2]=2*qz;  H_accel[1,3]=2*qy
        H_accel[2,0]=2*qw;  H_accel[2,1]=-2*qx; H_accel[2,2]=-2*qy; H_accel[2,3]=2*qz

        # 3 Measurement Residual (Innovation) y = z - h(x)
        # Difference between Real Sensor (z_accel) and Predicted Sensor (h_accel)
        y_a = z_accel - h_accel

        # 4 Innovation Covariance (S) and Kalman Gain (K)
        # S = H P H^T + R
        # K = P H^T S^-1  (Optimal weighting factor)
        S_a = H_accel @ self.P @ H_accel.T + self.R_accel
        K_a = self.P @ H_accel.T @ np.linalg.inv(S_a)

        # 5 State Update
        # x_new = x_old + K * residual
        self.x_state += K_a @ y_a

        # 6 Covariance Update
        # P_new = (I - K * H) * P_old
        # Reduces uncertainty because we added information from the sensor.
        self.P = (np.eye(7) - K_a @ H_accel) @ self.P

        # Renormalize quaternion to maintain valid rotation (Unit Norm constraint)
        self.x_state[:4] = self._normalize_quaternion(self.x_state[:4])

        
        # -----------------------------------------------------------
        # SECOND CORRECTION / UPDATE - MAGNETOMETER
        # Corrects Yaw (Heading) drift
        # -----------------------------------------------------------
        q = self.x_state[:4] 
        R_mat = self._quaternion_to_rotation_matrix(q)

        # 1 Predicted Measurement (h(x))
        # Project the Reference Magnetic Vector (World) into Body Frame
        h_mag = R_mat.T @ self.mag_ref
        
        mx_ref, my_ref, mz_ref = self.mag_ref
        qw, qx, qy, qz = q
        
        # 2 Jacobian H for Magnetometer
        # This is the derivative of the rotation matrix w.r.t the quaternion
        H_mag = np.zeros((3, 7))
        H_mag_q = 2 * np.array([
            [-my_ref*qz + mz_ref*qy, mz_ref*qz + my_ref*qy, mz_ref*qw + mx_ref*qz - my_ref*qx, my_ref*qw - mx_ref*qy - mz_ref*qx],
            [mx_ref*qz + mz_ref*qy - my_ref*qw, mz_ref*qy + my_ref*qz, mx_ref*qw - my_ref*qx + mz_ref*qz, mx_ref*qy - my_ref*qw + mz_ref*qx],
            [-mx_ref*qy + my_ref*qx - mz_ref*qw, my_ref*qy - mx_ref*qz, my_ref*qw + mx_ref*qy - mz_ref*qx, mx_ref*qw - my_ref*qz + mz_ref*qy]
        ])
        H_mag[:, :4] = H_mag_q
        
        # 3 EKF Update Sequence (same logic as Accel)
        y_m = z_mag - h_mag                         # Residual
        S_m = H_mag @ self.P @ H_mag.T + self.R_mag # Innovation Covariance
        K_m = self.P @ H_mag.T @ np.linalg.inv(S_m) # Kalman Gain

        self.x_state += K_m @ y_m                   # State Update
        self.P = (np.eye(7) - K_m @ H_mag) @ self.P # Covariance Update
        self.x_state[:4] = self._normalize_quaternion(self.x_state[:4])

        
        # -----------------------------------------------------------
        # DEAD RECKONING (Position Estimation)
        # Integrating Acceleration -> Velocity -> Position
        # -----------------------------------------------------------
        # 1 Normalize raw acceleration to G units
        accel_in_g = accel_raw / self.g_magnitude

        # 2 Rotate Acceleration from Sensor Frame to World Frame
        # v_world = R * v_body
        accel_world = R_mat @ accel_in_g

        # 3 Remove Gravity Vector
        linear_accel = accel_world - self.gravity_vector
        
        # 4 Stationary Detection (Zero Velocity Update logic)
        raw_accel_mag = np.linalg.norm(accel_raw)
        is_stationary = abs(raw_accel_mag - self.g_magnitude) < (self.stat_tolerance * self.g_magnitude)

        if is_stationary:
            linear_accel = np.zeros(3)
            self.vel *= 0.5 
            if np.linalg.norm(self.vel) < 0.05:
                self.vel = np.zeros(3)
        else:
            # Deadband Filter: Ignore noise below threshold
            if np.linalg.norm(linear_accel) < self.accel_threshold:
                linear_accel = np.zeros(3)
                self.vel *= 0.9 
        
        # 5 Integration: Accel -> Velocity
        accel_ms2 = linear_accel * 9.81
        if abs(accel_ms2[2]) < self.accel_threshold_z: 
            accel_ms2[2] = 0.0

        self.vel += accel_ms2 * dt

        # Apply Drag (Linear Damping) to prevent drift explosion
        self.vel[0] *= self.velocity_drag
        self.vel[1] *= self.velocity_drag
        self.vel[2] *= self.velocity_drag_z

        # 6 Integration: Velocity -> Position
        self.pos += self.vel * dt
        
        # ---- Better visualization options ----
        # Physics Constraint: Floor (Z cannot be negative)
        if self.use_floor and self.pos[2] < 0:
            self.pos[2] = 0
            self.vel[2] = 0
        
        # Visual Aid: Slowly return to center if stationary
        if self.return_center and is_stationary:
            self.pos *= 0.99

        # Prepare Output Data
        q = self.x_state[:4]
        variances = np.diag(self.P) # Diagonal elements represent variance of each state
        
        output = OrientationData(
            q[1], q[2], q[3], q[0],
            self.pos[0], self.pos[1], self.pos[2],
            *variances[:4]
        )
        self.mediator.publish(self.output_topic, output)