"""
madgwick_estimator.py

Implementação do Filtro Madgwick para fusão de sensores (9-DOF).
Inclui Dead Reckoning para estimativa de posição relativa.
"""

import time
import math
import numpy as np
import yaml

from data_types.orientation_data import OrientationData
from data_types.mpu_data import MpuData
from mediator import Mediator


class MadgwickEstimator:
    def __init__(self, mediator: Mediator, config_path="config.yaml"):
        self.mediator = mediator
        
        print(f"Madgwick: Carregando configurações de '{config_path}'...")
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)
            
        # Configurações de Tópicos
        self.input_topic = self.config['topics']['input_topic']
        self.output_topic = self.config['topics']['output_topic']
        self.mediator.subscribe(self.input_topic, self.handle_sensor_data)
        
        # --- Parâmetros Madgwick ---
        self.beta = self.config['madgwick']['beta']
        
        # Estado Inicial (Quaternião [qw, qx, qy, qz])
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        
        # Calibração Magnética
        self.mag_bias = np.array(self.config['magnetometer']['bias'])
        self.mag_scale = np.array(self.config['magnetometer']['scale'])
        
        # --- Configurações de Posição (Dead Reckoning) ---
        dr_conf = self.config['dead_reckoning']
        self.debug_mode = dr_conf.get('debug', False)
        self.accel_threshold = dr_conf['accel_threshold']
        self.calibration_frames = dr_conf['calibration_frames']
        self.drag_xy = dr_conf['drag']['xy']
        self.drag_z = dr_conf['drag']['z']
        self.return_center = dr_conf['return_to_center']
        self.center_speed = dr_conf.get('center_speed', 0.98)
        
        # Variáveis de Estado de Posição
        self.pos = np.array([0.0, 0.0, 0.0])
        self.vel = np.array([0.0, 0.0, 0.0])
        
        # Controle de Tempo e Calibração
        self.last_time = time.time()
        self.last_debug_time = 0
        self.calibration_samples = []
        self.g_magnitude = 9.81
        self.is_calibrating_g = True
        
        print(f"Madgwick Iniciado (Beta: {self.beta}). Calibrando gravidade...")

    def _quaternion_to_rotation_matrix(self, q):
        """Converte quaternião para Matriz de Rotação 3x3"""
        qw, qx, qy, qz = q
        return np.array([
            [1 - 2*qy**2 - 2*qz**2, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
            [2*qx*qy + 2*qz*qw, 1 - 2*qx**2 - 2*qz**2, 2*qy*qz - 2*qx*qw],
            [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx**2 - 2*qy**2]
        ])

    def handle_sensor_data(self, sensor_data: MpuData):
        if not sensor_data: return

        # 1. Cálculo do Delta Tempo (dt)
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        if dt > 0.1: dt = 0.01 # Proteção contra lag spike

        # 2. Extração e Normalização de Dados
        gx = math.radians(sensor_data.gx)
        gy = math.radians(sensor_data.gy)
        gz = math.radians(sensor_data.gz)

        ax, ay, az = sensor_data.ax, sensor_data.ay, sensor_data.az
        
        # --- Calibração Inicial da Gravidade ---
        if self.is_calibrating_g:
            norm_raw = math.sqrt(ax*ax + ay*ay + az*az)
            self.calibration_samples.append(norm_raw)
            if len(self.calibration_samples) > self.calibration_frames:
                self.g_magnitude = np.mean(self.calibration_samples)
                self.is_calibrating_g = False
                print(f"Gravidade Calibrada: {self.g_magnitude:.3f} (1G)")
            return

        # Normaliza Acelerômetro
        norm_a = math.sqrt(ax*ax + ay*ay + az*az)
        if norm_a == 0: return 
        ax_n, ay_n, az_n = ax/norm_a, ay/norm_a, az/norm_a

        # Magnetômetro (Aplica Hard/Soft Iron)
        mx = (sensor_data.mx - self.mag_bias[0]) * self.mag_scale[0]
        my = (sensor_data.my - self.mag_bias[1]) * self.mag_scale[1]
        mz = (sensor_data.mz - self.mag_bias[2]) * self.mag_scale[2]
        
        # Ajuste de Eixos (Padrão: Y, X, -Z para alinhar NED)
        mx_s, my_s, mz_s = my, mx, -mz 
        
        norm_m = math.sqrt(mx_s*mx_s + my_s*my_s + mz_s*mz_s)
        if norm_m == 0: return
        mx_s /= norm_m; my_s /= norm_m; mz_s /= norm_m

        # 3. ALGORITMO MADGWICK
        qw, qx, qy, qz = self.q
        
        # Variáveis auxiliares para otimização
        _2qw = 2.0 * qw; _2qx = 2.0 * qx; _2qy = 2.0 * qy; _2qz = 2.0 * qz

        # A. Predição pelo Giroscópio
        q_dot_w = 0.5 * (-qx*gx - qy*gy - qz*gz)
        q_dot_x = 0.5 * ( qw*gx + qy*gz - qz*gy)
        q_dot_y = 0.5 * ( qw*gy - qx*gz + qz*gx)
        q_dot_z = 0.5 * ( qw*gz + qx*gy - qy*gx)

        # B. Correção pelo Gradiente Descendente
        # Rotação do fluxo magnético
        hx = mx_s * (1 - 2*qy*qy - 2*qz*qz) + my_s * (2*qx*qy - 2*qw*qz) + mz_s * (2*qx*qz + 2*qw*qy)
        hy = mx_s * (2*qx*qy + 2*qw*qz) + my_s * (1 - 2*qx*qx - 2*qz*qz) + mz_s * (2*qy*qz - 2*qw*qx)
        hz = mx_s * (2*qx*qz - 2*qw*qy) + my_s * (2*qy*qz + 2*qw*qx) + mz_s * (1 - 2*qx*qx - 2*qy*qy)
        
        bx = math.sqrt(hx*hx + hy*hy)
        bz = hz 
        _2bx = 2.0 * bx; _2bz = 2.0 * bz

        # Gradiente do erro (Função Objetivo f)
        s_w = -_2qy * (2.0 * (qx*qz - qw*qy) - ax_n) + _2qx * (2.0 * (qw*qx + qy*qz) - ay_n) - _2bz * qy * (_2bx * (0.5 - qy*qy - qz*qz) + _2bz * (qx*qz - qw*qy) - mx_s) + (-_2bx * qz + _2bz * qx) * (_2bx * (qx*qy - qw*qz) + _2bz * (qw*qx + qy*qz) - my_s) + _2bx * qy * (_2bx * (qx*qz + qw*qy) + _2bz * (0.5 - qx*qx - qy*qy) - mz_s)
        s_x = _2qz * (2.0 * (qx*qz - qw*qy) - ax_n) + _2qw * (2.0 * (qw*qx + qy*qz) - ay_n) - 4.0 * qx * (1 - 2.0 * (qx*qx + qy*qy) - az_n) + _2bz * qz * (_2bx * (0.5 - qy*qy - qz*qz) + _2bz * (qx*qz - qw*qy) - mx_s) + (_2bx * qy + _2bz * qw) * (_2bx * (qx*qy - qw*qz) + _2bz * (qw*qx + qy*qz) - my_s) + (_2bx * qz - _2bz * 4.0 * qx) * (_2bx * (qx*qz + qw*qy) + _2bz * (0.5 - qx*qx - qy*qy) - mz_s)
        s_y = -_2qw * (2.0 * (qx*qz - qw*qy) - ax_n) + _2qz * (2.0 * (qw*qx + qy*qz) - ay_n) - 4.0 * qy * (1 - 2.0 * (qx*qx + qy*qy) - az_n) + (-_2bx * 4.0 * qy - _2bz * 2.0 * qz) * (_2bx * (0.5 - qy*qy - qz*qz) + _2bz * (qx*qz - qw*qy) - mx_s) + (_2bx * qx + _2bz * qz) * (_2bx * (qx*qy - qw*qz) + _2bz * (qw*qx + qy*qz) - my_s) + (_2bx * qw - _2bz * 4.0 * qy) * (_2bx * (qx*qz + qw*qy) + _2bz * (0.5 - qx*qx - qy*qy) - mz_s)
        s_z = _2qx * (2.0 * (qx*qz - qw*qy) - ax_n) + _2qy * (2.0 * (qw*qx + qy*qz) - ay_n) + (-_2bx * 4.0 * qz + _2bz * 2.0 * qy) * (_2bx * (0.5 - qy*qy - qz*qz) + _2bz * (qx*qz - qw*qy) - mx_s) + (-_2bx * qw + _2bz * qx) * (_2bx * (qx*qy - qw*qz) + _2bz * (qw*qx + qy*qz) - my_s) + (_2bx * qx + _2bz * 4.0 * qz) * (_2bx * (qx*qz + qw*qy) + _2bz * (0.5 - qx*qx - qy*qy) - mz_s)
        
        norm_s = math.sqrt(s_w*s_w + s_x*s_x + s_y*s_y + s_z*s_z)
        if norm_s > 0:
            s_w /= norm_s; s_x /= norm_s; s_y /= norm_s; s_z /= norm_s

        # Aplica o Beta (Feedback)
        q_dot_w -= self.beta * s_w
        q_dot_x -= self.beta * s_x
        q_dot_y -= self.beta * s_y
        q_dot_z -= self.beta * s_z

        # Integração e Normalização Final
        self.q[0] += q_dot_w * dt
        self.q[1] += q_dot_x * dt
        self.q[2] += q_dot_y * dt
        self.q[3] += q_dot_z * dt
        
        self.q /= np.linalg.norm(self.q)

        # 4. DEAD RECKONING (Posição)
        # Normaliza accel bruta para Gs
        accel_vec_raw = np.array([ax, ay, az]) / self.g_magnitude
        
        # Rotaciona para o Mundo
        R = self._quaternion_to_rotation_matrix(self.q)
        accel_world = R @ accel_vec_raw
        
        # Remove Gravidade (Z = 1.0)
        accel_world[2] -= 1.0
        
        # Filtro de Limiar (Deadband)
        lin_accel = np.zeros(3)
        if abs(accel_world[0]) > self.accel_threshold: lin_accel[0] = accel_world[0]
        if abs(accel_world[1]) > self.accel_threshold: lin_accel[1] = accel_world[1]
        if abs(accel_world[2]) > (self.accel_threshold * 1.5): lin_accel[2] = accel_world[2]
        
        # Integração (Velocidade)
        self.vel += lin_accel * 9.81 * dt
        
        # Arrasto (Drag)
        self.vel[0] *= self.drag_xy
        self.vel[1] *= self.drag_xy
        self.vel[2] *= self.drag_z
        
        # ZUPT (Zero Velocity Update) se estiver quase parado
        accel_mag_now = math.sqrt(ax*ax + ay*ay + az*az)
        if abs(accel_mag_now - self.g_magnitude) < 0.05 * self.g_magnitude:
             self.vel *= 0.8

        # Integração (Posição)
        self.pos += self.vel * dt
        
        # Retorno ao Centro (Recurso solicitado)
        if self.return_center and np.linalg.norm(self.vel) < 0.05:
            self.pos *= self.center_speed

        # 5. DEBUG e PUBLICAÇÃO
        if self.debug_mode and (current_time - self.last_debug_time > 0.5):
            self.last_debug_time = current_time
            # Converte para Euler para leitura humana
            yaw = math.degrees(math.atan2(2.0*(self.q[1]*self.q[2] + self.q[0]*self.q[3]), 
                                          self.q[0]**2 + self.q[1]**2 - self.q[2]**2 - self.q[3]**2))
            print(f"DEBUG: Yaw={yaw:.1f}° | Pos=({self.pos[0]:.2f}, {self.pos[1]:.2f}, {self.pos[2]:.2f})")

        output = OrientationData(
            self.q[1], self.q[2], self.q[3], self.q[0], 
            self.pos[0], self.pos[1], self.pos[2],
            0, 0, 0, 0       
        )
        self.mediator.publish(self.output_topic, output)