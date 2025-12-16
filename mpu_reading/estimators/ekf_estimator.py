"""
ekf_estimator.py

Filtro de Kalman Estendido (EKF) para fusão de sensores (9-DOF).
Atualizado para usar a estrutura unificada de 'config.yaml' e Dead Reckoning aprimorado.
"""

import time
import math
import yaml
import numpy as np

from data_types.orientation_data import OrientationData
from data_types.mpu_data import MpuData
from mediator import Mediator


class EkfEstimator:
    """
    Implementação acadêmica de um EKF de 7 estados (Quaternião + Bias do Giroscópio).
    """
    def __init__(self, mediator: Mediator, config_path="config.yaml"):
        self.mediator = mediator
        
        print(f"EKF: Carregando configurações de '{config_path}'...")
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)
            
        # Topics
        self.input_topic = self.config['topics']['input_topic']
        self.output_topic = self.config['topics']['output_topic']
        self.mediator.subscribe(self.input_topic, self.handle_sensor_data)
        
        # --- Configuração EKF ---
        ekf_conf = self.config['ekf']
        self.dt_min = ekf_conf['dt_min']
        
        # Estado Inicial: [qw, qx, qy, qz, bx, by, bz]
        self.x_state = np.zeros(7)
        self.x_state[0] = 1.0 
        
        # Covariância Inicial (P)
        p_orient = ekf_conf['initial_covariance']['orientation_scale']
        p_bias = ekf_conf['initial_covariance']['bias_scale']
        self.P = np.eye(7)
        self.P[:4, :4] *= p_orient
        self.P[4:, 4:] *= p_bias

        # Matrizes de Ruído (Q e R)
        self.R_accel = np.eye(3) * ekf_conf['measurement_noise']['accel_factor']
        self.R_mag = np.eye(3) * ekf_conf['measurement_noise']['mag_factor']
        self.Q_noise = np.diag(ekf_conf['process_noise'])

        # --- Calibração Magnética (Global) ---
        self.mag_bias = np.array(self.config['magnetometer']['bias'])
        self.mag_scale = np.array(self.config['magnetometer']['scale'])
        self.mag_ref = None 

        # --- Configurações de Posição (Dead Reckoning) ---
        dr_conf = self.config['dead_reckoning']
        self.debug_mode = dr_conf.get('debug', False)
        self.accel_threshold = dr_conf['accel_threshold']
        self.calibration_frames = dr_conf['calibration_frames']
        self.drag_xy = dr_conf['drag']['xy']
        self.drag_z = dr_conf['drag']['z']
        self.return_center = dr_conf['return_to_center']
        self.center_speed = dr_conf.get('center_speed', 0.98)
        self.gravity_vector = np.array([0.0, 0.0, 1.0])

        # Variáveis de Estado de Posição
        self.pos = np.array([0.0, 0.0, 0.0]) 
        self.vel = np.array([0.0, 0.0, 0.0]) 

        # Controle Interno
        self.calibration_samples = []
        self.g_magnitude = 9.81
        self.is_calibrating_g = True
        self.last_time = time.time()
        self.last_debug_time = 0
        
        print("EKF Initialized. Calibrando gravidade...")

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

    def _get_yaw_from_quat(self, q):
        """Helper para Debug: Extrai Yaw (em graus) do quaternião."""
        qw, qx, qy, qz = q
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return math.degrees(yaw)

    def handle_sensor_data(self, sensor_data: MpuData):
        if not sensor_data: return

        current_time = time.time()
        dt = current_time - self.last_time
        if dt <= self.dt_min: return
        self.last_time = current_time

        accel_raw = np.array([sensor_data.ax, sensor_data.ay, sensor_data.az])

        # --- 1. Calibração de Gravidade ---
        if self.is_calibrating_g:
            self.calibration_samples.append(np.linalg.norm(accel_raw))
            if len(self.calibration_samples) > self.calibration_frames:
                self.g_magnitude = np.mean(self.calibration_samples)
                self.is_calibrating_g = False
                print(f"EKF: Gravidade Calibrada: {self.g_magnitude:.2f} units")
            return 

        # --- 2. Preparação dos Dados ---
        # Magnetômetro com calibração Hard/Soft Iron
        mx = (sensor_data.mx - self.mag_bias[0]) * self.mag_scale[0]
        my = (sensor_data.my - self.mag_bias[1]) * self.mag_scale[1]
        mz = (sensor_data.mz - self.mag_bias[2]) * self.mag_scale[2]
        
        # --- EIXOS (Igual ao Madgwick para comparação justa) ---
        # Trocando X por Y para alinhar com padrão NED comum em módulos MPU9250
        mx_s, my_s, mz_s = my, mx, -mz 
        
        # Vetor Mag Normalizado
        mag_vec = np.array([mx_s, my_s, mz_s])
        mag_norm = np.linalg.norm(mag_vec)
        z_mag = mag_vec / mag_norm if mag_norm > 0 else np.zeros(3)

        # Vetor Accel Normalizado
        accel_norm = np.linalg.norm(accel_raw)
        z_accel = accel_raw / accel_norm if accel_norm > 0 else np.zeros(3)

        # Inicialização da Referência Magnética (Plana)
        if self.mag_ref is None:
            heading = math.atan2(z_mag[1], z_mag[0])
            # Força referência Z=0 para evitar instabilidade de Dip Angle
            self.mag_ref = np.array([math.cos(heading), math.sin(heading), 0.0])
            print(f"EKF Ref Definida (Plana): {self.mag_ref}")
            return


        # --- 3. PREDIÇÃO EKF (Modelo do Giroscópio) ---
        gyro = np.array([math.radians(x) for x in [sensor_data.gx, sensor_data.gy, sensor_data.gz]])
        q = self.x_state[:4]; b = self.x_state[4:]
        w = gyro - b # Corrige com o Bias estimado

        Omega = np.array([[0, -w[0], -w[1], -w[2]], [w[0], 0, w[2], -w[1]], [w[1], -w[2], 0, w[0]], [w[2], w[1], -w[0], 0]])
        
        # Integração Numérica
        q_new = q + 0.5 * Omega @ q * dt
        self.x_state[:4] = self._normalize_quaternion(q_new)
        
        # Jacobianas F e G
        F = np.eye(7); F[:4, :4] += 0.5 * Omega * dt
        Xi = 0.5 * np.array([[-q[1], -q[2], -q[3]], [q[0], -q[3], q[2]], [q[3], q[0], -q[1]], [-q[2], q[1], q[0]]])
        F[:4, 4:] = -Xi * dt
        G = np.zeros((7, 6)); G[:4, :3] = -Xi * dt; G[4:, 3:] = np.eye(3) * dt
        
        # Predição da Covariância
        self.P = F @ self.P @ F.T + (G @ self.Q_noise @ G.T)

        
        # --- 4. CORREÇÃO 1: ACELERÔMETRO ---
        qw, qx, qy, qz = self.x_state[:4]
        # Gravidade estimada no corpo
        h_accel = np.array([2*(qx*qz - qw*qy), 2*(qy*qz + qw*qx), qw**2 - qx**2 - qy**2 + qz**2])

        H_accel = np.zeros((3, 7))
        H_accel[0,0]=-2*qy; H_accel[0,1]=2*qz; H_accel[0,2]=-2*qw; H_accel[0,3]=2*qx
        H_accel[1,0]=2*qx;  H_accel[1,1]=2*qw; H_accel[1,2]=2*qz;  H_accel[1,3]=2*qy
        H_accel[2,0]=2*qw;  H_accel[2,1]=-2*qx; H_accel[2,2]=-2*qy; H_accel[2,3]=2*qz

        y_a = z_accel - h_accel # Resíduo
        S_a = H_accel @ self.P @ H_accel.T + self.R_accel
        K_a = self.P @ H_accel.T @ np.linalg.inv(S_a)
        
        self.x_state += K_a @ y_a
        self.P = (np.eye(7) - K_a @ H_accel) @ self.P
        self.x_state[:4] = self._normalize_quaternion(self.x_state[:4])

        
        # --- 5. CORREÇÃO 2: MAGNETÔMETRO ---
        q = self.x_state[:4] 
        R_mat = self._quaternion_to_rotation_matrix(q)
        
        # Predição: Onde o Norte (mag_ref) deveria estar no corpo?
        h_mag = R_mat.T @ self.mag_ref 
        
        mx_ref, my_ref, mz_ref = self.mag_ref
        qw, qx, qy, qz = q
        H_mag = np.zeros((3, 7))
        # Derivadas parciais da matriz de rotação transposta
        H_mag_q = 2 * np.array([
            [-my_ref*qz + mz_ref*qy, mz_ref*qz + my_ref*qy, mz_ref*qw + mx_ref*qz - my_ref*qx, my_ref*qw - mx_ref*qy - mz_ref*qx],
            [mx_ref*qz + mz_ref*qy - my_ref*qw, mz_ref*qy + my_ref*qz, mx_ref*qw - my_ref*qx + mz_ref*qz, mx_ref*qy - my_ref*qw + mz_ref*qx],
            [-mx_ref*qy + my_ref*qx - mz_ref*qw, my_ref*qy - mx_ref*qz, my_ref*qw + mx_ref*qy - mz_ref*qx, mx_ref*qw - my_ref*qz + mz_ref*qy]
        ])
        H_mag[:, :4] = H_mag_q
        
        y_m = z_mag - h_mag # Resíduo
        S_m = H_mag @ self.P @ H_mag.T + self.R_mag
        K_m = self.P @ H_mag.T @ np.linalg.inv(S_m)
        
        correction = K_m @ y_m
        self.x_state += correction
        self.P = (np.eye(7) - K_m @ H_mag) @ self.P
        self.x_state[:4] = self._normalize_quaternion(self.x_state[:4])


        # --- 6. DEBUG (Igual ao solicitado) ---
        if self.debug_mode and (current_time - self.last_debug_time > 0.5):
            self.last_debug_time = current_time
            
            # Dados para análise
            est_yaw = self._get_yaw_from_quat(self.x_state[:4])
            raw_heading = math.degrees(math.atan2(z_mag[1], z_mag[0]))
            corr_mag = np.linalg.norm(correction[:4])
            gain_avg = np.mean(np.abs(K_m))
            
            print(f"\n--- EKF DEBUG ---")
            print(f"Yaw Est: {est_yaw:6.1f}° | Raw Mag: {raw_heading:6.1f}°")
            print(f"Resíduo Mag: [{y_m[0]:.2f}, {y_m[1]:.2f}, {y_m[2]:.2f}]")
            print(f"Ganho K (Médio): {gain_avg:.5f} | Correção: {corr_mag:.5f}")
            print(f"Posição: ({self.pos[0]:.2f}, {self.pos[1]:.2f}, {self.pos[2]:.2f})")


        # --- 7. DEAD RECKONING (Posição) ---
        # Normaliza aceleração para Gs
        accel_in_g = accel_raw / self.g_magnitude

        # Rotaciona para o Mundo
        R_curr = self._quaternion_to_rotation_matrix(self.x_state[:4])
        accel_world = R_curr @ accel_in_g

        # Remove gravidade (Z=1.0)
        accel_world[2] -= 1.0
        
        # Deadband / Threshold
        linear_accel = np.zeros(3)
        if abs(accel_world[0]) > self.accel_threshold: linear_accel[0] = accel_world[0]
        if abs(accel_world[1]) > self.accel_threshold: linear_accel[1] = accel_world[1]
        if abs(accel_world[2]) > (self.accel_threshold * 1.5): linear_accel[2] = accel_world[2]
        
        # Integração Velocidade
        self.vel += linear_accel * 9.81 * dt

        # Arrasto
        self.vel[0] *= self.drag_xy
        self.vel[1] *= self.drag_xy
        self.vel[2] *= self.drag_z
        
        # ZUPT (Zero Velocity Update)
        raw_mag_accel = np.linalg.norm(accel_raw)
        if abs(raw_mag_accel - self.g_magnitude) < 0.05 * self.g_magnitude:
            self.vel *= 0.8
            
        # Retorno ao Centro
        if self.return_center and np.linalg.norm(self.vel) < 0.05:
            self.pos *= self.center_speed

        # Integração Posição
        self.pos += self.vel * dt
        
        # --- Publicação ---
        q = self.x_state[:4]
        variances = np.diag(self.P)
        output = OrientationData(
            q[1], q[2], q[3], q[0], 
            self.pos[0], self.pos[1], self.pos[2],
            *variances[:4]
        )
        self.mediator.publish(self.output_topic, output)