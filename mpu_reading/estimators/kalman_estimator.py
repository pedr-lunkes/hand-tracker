"""
imu_kalman_estimator.py

Filtro de Kalman Estendido (EKF) para 6-DOF (Apenas Giroscópio + Acelerômetro).
Estabiliza Pitch/Roll, mas permite drift no Yaw.
"""

import time
import math
import yaml
import numpy as np

from data_types.orientation_data import OrientationData
from data_types.mpu_data import MpuData
from mediator import Mediator

class KalmanEstimator:
    def __init__(self, mediator: Mediator, config_path="config.yaml"):
        self.mediator = mediator
        
        print(f"IMU Kalman (6-DOF): Carregando configurações de '{config_path}'...")
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)
            
        self.input_topic = self.config['topics']['input_topic']
        self.output_topic = self.config['topics']['output_topic']
        self.mediator.subscribe(self.input_topic, self.handle_sensor_data)
        
        # --- Configuração EKF ---
        kf_conf = self.config['kalman6d']
        self.dt_min = kf_conf['dt_min']
        
        # Estado Inicial: [qw, qx, qy, qz, bx, by, bz]
        self.x_state = np.zeros(7)
        self.x_state[0] = 1.0 # Quaternião Identidade
        
        # Covariância Inicial (P)
        p_orient = kf_conf['initial_covariance']['orientation_scale']
        p_bias = kf_conf['initial_covariance']['bias_scale']
        self.P = np.eye(7)
        self.P[:4, :4] *= p_orient
        self.P[4:, 4:] *= p_bias

        # Matrizes de Ruído
        self.R_accel = np.eye(3) * kf_conf['measurement_noise']['accel_factor']
        self.Q_noise = np.diag(kf_conf['process_noise'])

        # --- Dead Reckoning (Posição) ---
        dr_conf = self.config['dead_reckoning']
        self.enable_pos = dr_conf.get('enable', False)
        self.debug_mode = dr_conf.get('debug', False)
        self.accel_threshold = dr_conf['accel_threshold']
        self.calibration_frames = dr_conf['calibration_frames']
        self.drag_xy = dr_conf['drag']['xy']
        self.drag_z = dr_conf['drag']['z']
        self.return_center = dr_conf['return_to_center']
        self.center_speed = dr_conf.get('center_speed', 0.98)

        self.pos = np.array([0.0, 0.0, 0.0])
        self.vel = np.array([0.0, 0.0, 0.0])
        self.gravity_vector = np.array([0.0, 0.0, 1.0]) # Gravidade teórica normalizada

        self.calibration_samples = []
        self.g_magnitude = 9.81
        self.is_calibrating_g = True
        self.last_time = time.time()
        self.last_debug_time = 0
        
        print("IMU Kalman 6-DOF Iniciado.")

    def _normalize_quaternion(self, q):
        norm = np.linalg.norm(q)
        return q / norm if norm > 0 else np.array([1.0, 0.0, 0.0, 0.0])

    def _quaternion_to_rotation_matrix(self, q):
        qw, qx, qy, qz = q
        return np.array([
            [1 - 2*qy**2 - 2*qz**2, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
            [2*qx*qy + 2*qz*qw, 1 - 2*qx**2 - 2*qz**2, 2*qy*qz - 2*qx*qw],
            [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx**2 - 2*qy**2]
        ])

    def handle_sensor_data(self, sensor_data: MpuData):
        if not sensor_data: return

        current_time = time.time()
        dt = current_time - self.last_time
        if dt <= self.dt_min: return
        self.last_time = current_time

        accel_raw = np.array([sensor_data.ax, sensor_data.ay, sensor_data.az])

        # 1. Calibração G
        if self.is_calibrating_g:
            self.calibration_samples.append(np.linalg.norm(accel_raw))
            if len(self.calibration_samples) > self.calibration_frames:
                avg_g = np.mean(self.calibration_samples)
                self.g_magnitude = avg_g if 0.5 < avg_g < 1.5 else 1.0
                self.is_calibrating_g = False
                print(f"IMU Kalman: G calibrado em {self.g_magnitude:.3f}")
            return 

        # 2. Dados Normalizados (Não precisamos de calibração magnética aqui)
        accel_norm = np.linalg.norm(accel_raw)
        z_accel = accel_raw / accel_norm if accel_norm > 0 else np.zeros(3)

        # --- 3. PREDIÇÃO (Giroscópio) ---
        gyro = np.array([math.radians(x) for x in [sensor_data.gx, sensor_data.gy, sensor_data.gz]])
        q = self.x_state[:4]; b = self.x_state[4:]
        w = gyro - b 

        Omega = np.array([[0, -w[0], -w[1], -w[2]], [w[0], 0, w[2], -w[1]], [w[1], -w[2], 0, w[0]], [w[2], w[1], -w[0], 0]])
        
        # Integração do Estado
        q_new = q + 0.5 * Omega @ q * dt
        self.x_state[:4] = self._normalize_quaternion(q_new)
        
        # Jacobiana F
        F = np.eye(7); F[:4, :4] += 0.5 * Omega * dt
        Xi = 0.5 * np.array([[-q[1], -q[2], -q[3]], [q[0], -q[3], q[2]], [q[3], q[0], -q[1]], [-q[2], q[1], q[0]]])
        F[:4, 4:] = -Xi * dt
        
        # Predição da Covariância P
        G = np.zeros((7, 6)); G[:4, :3] = -Xi * dt; G[4:, 3:] = np.eye(3) * dt
        self.P = F @ self.P @ F.T + (G @ self.Q_noise @ G.T)

        
        # --- 4. CORREÇÃO (Apenas Acelerômetro) ---
        qw, qx, qy, qz = self.x_state[:4]
        
        # Modelo de Medição h(x): Onde a gravidade [0,0,1] estaria no corpo?
        # É a última coluna da Matriz de Rotação Transposta
        h_accel = np.array([
            2*(qx*qz - qw*qy), 
            2*(qy*qz + qw*qx), 
            qw**2 - qx**2 - qy**2 + qz**2
        ])

        # Jacobiana H (Derivada de h em relação aos quaterniões)
        H_accel = np.zeros((3, 7))
        H_accel[0,0]=-2*qy; H_accel[0,1]=2*qz; H_accel[0,2]=-2*qw; H_accel[0,3]=2*qx
        H_accel[1,0]=2*qx;  H_accel[1,1]=2*qw; H_accel[1,2]=2*qz;  H_accel[1,3]=2*qy
        H_accel[2,0]=2*qw;  H_accel[2,1]=-2*qx; H_accel[2,2]=-2*qy; H_accel[2,3]=2*qz

        # Passo de Atualização Kalman
        y_a = z_accel - h_accel # Inovação (Resíduo)
        S_a = H_accel @ self.P @ H_accel.T + self.R_accel
        K_a = self.P @ H_accel.T @ np.linalg.inv(S_a) # Ganho
        
        self.x_state += K_a @ y_a
        self.P = (np.eye(7) - K_a @ H_accel) @ self.P
        self.x_state[:4] = self._normalize_quaternion(self.x_state[:4])


        # --- 5. DEAD RECKONING ---
        if self.enable_pos:
            accel_in_g = accel_raw / self.g_magnitude
            R_curr = self._quaternion_to_rotation_matrix(self.x_state[:4])
            accel_world = R_curr @ accel_in_g
            accel_world[2] -= 1.0
            
            linear_accel = np.zeros(3)
            if abs(accel_world[0]) > self.accel_threshold: linear_accel[0] = accel_world[0]
            if abs(accel_world[1]) > self.accel_threshold: linear_accel[1] = accel_world[1]
            if abs(accel_world[2]) > (self.accel_threshold * 1.5): linear_accel[2] = accel_world[2]
            
            self.vel += linear_accel * 9.81 * dt
            self.vel[0] *= self.drag_xy; self.vel[1] *= self.drag_xy; self.vel[2] *= self.drag_z
            
            raw_mag = np.linalg.norm(accel_raw)
            if abs(raw_mag - self.g_magnitude) < 0.05 * self.g_magnitude: self.vel *= 0.8
            
            self.pos += self.vel * dt
            if self.return_center and np.linalg.norm(self.vel) < 0.05: self.pos *= self.center_speed
        else:
            self.pos = np.zeros(3)
            self.vel = np.zeros(3)

        # --- Debug ---
        if self.debug_mode and (current_time - self.last_debug_time > 0.5):
            self.last_debug_time = current_time
            pitch = math.degrees(math.asin(2.0 * (qw*qy - qz*qx)))
            roll  = math.degrees(math.atan2(2.0*(qw*qx + qy*qz), 1.0 - 2.0*(qx*qx + qy*qy)))
            print(f"IMU Kalman: Pitch={pitch:.1f}° | Roll={roll:.1f}° (Yaw Drift Free)")

        # Envia
        q = self.x_state[:4]
        output = OrientationData(
            q[1], q[2], q[3], q[0], 
            self.pos[0], self.pos[1], self.pos[2],
            0, 0, 0, 0       
        )
        self.mediator.publish(self.output_topic, output)