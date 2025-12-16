"""
ekf_estimator.py

Implementação de um Filtro de Kalman Estendido (EKF) para fusão de sensores 9-DOF.
Este estimador combina dados de Giroscópio, Acelerômetro e Magnetômetro para estimar
a orientação (Quaterniões) e o bias do giroscópio.

Estrutura do Filtro:
1. Predição (Time Update): Integração da velocidade angular do giroscópio.
2. Correção 1 (Measurement Update): Vetor Gravidade (Acelerômetro) corrige Pitch/Roll.
3. Correção 2 (Measurement Update): Vetor Norte (Magnetômetro) corrige Yaw.
"""

import time
import math
import yaml
import numpy as np

# Importação dos tipos de dados definidos no projeto
from data_types.orientation_data import OrientationData
from data_types.mpu_data import MpuData
from mediator import Mediator


class EkfEstimator:
    def __init__(self, mediator: Mediator, config_path="config.yaml"):
        self.mediator = mediator
        
        print(f"EKF: Carregando configurações de '{config_path}'...")
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)
            
        # Configuração de Tópicos (Pub/Sub)
        self.input_topic = self.config['topics']['input_topic']
        self.output_topic = self.config['topics']['output_topic']
        self.mediator.subscribe(self.input_topic, self.handle_sensor_data)
        
        # --- Parâmetros do EKF ---
        ekf_conf = self.config['ekf']
        self.dt_min = ekf_conf['dt_min']
        
        # Vetor de Estado Inicial (x)
        # Inicializa com quaternião identidade (rotação nula) e bias zero.
        self.x_state = np.zeros(7)
        self.x_state[0] = 1.0 
        
        # Matriz de Covariância Inicial (P)
        # Representa a incerteza inicial sobre o estado.
        # Valores altos na diagonal indicam baixa confiança inicial.
        p_orient = ekf_conf['initial_covariance']['orientation_scale']
        p_bias = ekf_conf['initial_covariance']['bias_scale']
        self.P = np.eye(7)
        self.P[:4, :4] *= p_orient # Incerteza da orientação
        self.P[4:, 4:] *= p_bias   # Incerteza do bias

        # Matrizes de Ruído
        # R: Ruído de Medição (Incerteza dos sensores Acel/Mag)
        self.R_accel = np.eye(3) * ekf_conf['measurement_noise']['accel_factor']
        self.R_mag = np.eye(3) * ekf_conf['measurement_noise']['mag_factor']
        
        # Q: Ruído de Processo (Incerteza do modelo de predição/Giroscópio)
        self.Q_noise = np.diag(ekf_conf['process_noise'])

        # --- Calibração Magnética ---
        # Carrega parâmetros de Hard Iron (Bias) e Soft Iron (Scale)
        self.mag_bias = np.array(self.config['magnetometer']['bias'])
        self.mag_scale = np.array(self.config['magnetometer']['scale'])
        self.mag_ref = None # Será definido na primeira iteração

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

        # Variáveis de Estado de Posição
        self.pos = np.array([0.0, 0.0, 0.0]) 
        self.vel = np.array([0.0, 0.0, 0.0]) 

        # Controle Interno e Calibração G
        self.calibration_samples = []
        self.g_magnitude = 9.81
        self.is_calibrating_g = True
        self.last_time = time.time()
        self.last_debug_time = 0
        
        print("EKF Inicializado. Calibrando gravidade...")

    def _normalize_quaternion(self, q):
        """Normaliza o quaternião para garantir magnitude unitária."""
        norm = np.linalg.norm(q)
        return q / norm if norm > 0 else np.array([1.0, 0.0, 0.0, 0.0])

    def _quaternion_to_rotation_matrix(self, q):
        """Converte Quaternião para Matriz de Rotação (DCM - Direction Cosine Matrix)."""
        qw, qx, qy, qz = q
        return np.array([
            [1 - 2*qy**2 - 2*qz**2, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
            [2*qx*qy + 2*qz*qw, 1 - 2*qx**2 - 2*qz**2, 2*qy*qz - 2*qx*qw],
            [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx**2 - 2*qy**2]
        ])

    def _get_yaw_from_quat(self, q):
        """Função auxiliar para debug: extrai Yaw em graus."""
        qw, qx, qy, qz = q
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        return math.degrees(math.atan2(siny_cosp, cosy_cosp))

    def handle_sensor_data(self, sensor_data: MpuData):
        if not sensor_data: return

        # Cálculo do passo de tempo (dt)
        current_time = time.time()
        dt = current_time - self.last_time
        if dt <= self.dt_min: return # Evita atualizações excessivamente frequentes
        self.last_time = current_time

        accel_raw = np.array([sensor_data.ax, sensor_data.ay, sensor_data.az])

        # --- Rotina de Calibração da Gravidade ---
        if self.is_calibrating_g:
            self.calibration_samples.append(np.linalg.norm(accel_raw))
            if len(self.calibration_samples) > self.calibration_frames:
                self.g_magnitude = np.mean(self.calibration_samples)
                # Proteção contra valores espúrios
                self.g_magnitude = self.g_magnitude if 0.5 < self.g_magnitude < 1.5 else 1.0
                self.is_calibrating_g = False
                print(f"EKF: Gravidade Calibrada: {self.g_magnitude:.3f}g")
            return 

        # --- Pré-processamento dos Dados ---
        
        # Magnetômetro: Aplica calibração e ajusta eixos
        mx = (sensor_data.mx - self.mag_bias[0]) * self.mag_scale[0]
        my = (sensor_data.my - self.mag_bias[1]) * self.mag_scale[1]
        mz = (sensor_data.mz - self.mag_bias[2]) * self.mag_scale[2]
        
        # Mapeamento de eixos (NED Standard: X=Norte, Y=Leste, Z=Baixo)
        # O hardware MPU9250 geralmente requer troca de eixos no magnetômetro.
        mx_s, my_s, mz_s = my, mx, -mz 
        
        # Normalização dos vetores de medição (z)
        mag_vec = np.array([mx_s, my_s, mz_s])
        mag_norm = np.linalg.norm(mag_vec)
        z_mag = mag_vec / mag_norm if mag_norm > 0 else np.zeros(3)

        accel_norm = np.linalg.norm(accel_raw)
        z_accel = accel_raw / accel_norm if accel_norm > 0 else np.zeros(3)

        # Definição do vetor de referência magnética (Norte Inicial)
        if self.mag_ref is None:
            # Assume que o eixo X aponta para o norte magnético local no início
            heading = math.atan2(z_mag[1], z_mag[0])
            # Força componente Z=0 para desacoplar dip angle
            self.mag_ref = np.array([math.cos(heading), math.sin(heading), 0.0])
            print(f"EKF: Referência Magnética definida: {self.mag_ref}")
            return

        # ====================================================================
        # PREDIÇÃO (Time Update)
        # Propaga o estado usando o modelo cinemático do giroscópio.
        # ====================================================================
        
        # Obter velocidade angular e corrigir com o Bias estimado anteriormente
        gyro = np.array([math.radians(x) for x in [sensor_data.gx, sensor_data.gy, sensor_data.gz]])
        q = self.x_state[:4] # Estado atual (Quaternião)
        b = self.x_state[4:] # Estado atual (Bias)
        w = gyro - b         # Velocidade angular corrigida

        # Matriz de transição Omega
        # Relaciona a derivada do quaternião com a velocidade angular.
        # dq/dt = 0.5 * Omega * q
        Omega = np.array([
            [0,    -w[0], -w[1], -w[2]],
            [w[0],  0,     w[2], -w[1]],
            [w[1], -w[2],  0,     w[0]],
            [w[2],  w[1], -w[0],  0   ]
        ])
        
        # Integração Numérica (Euler)
        # x_k = x_{k-1} + (dx/dt * dt)
        q_new = q + 0.5 * Omega @ q * dt
        self.x_state[:4] = self._normalize_quaternion(q_new)
        
        # Cálculo da Jacobiana F (Linearização do sistema)
        # F = df/dx (Derivada da função de transição em relação ao estado)
        F = np.eye(7)
        F[:4, :4] += 0.5 * Omega * dt # Parte da orientação

        # Derivada em relação ao Bias envolve matriz Xi
        Xi = 0.5 * np.array([
            [-q[1], -q[2], -q[3]],
            [ q[0], -q[3],  q[2]],
            [ q[3],  q[0], -q[1]],
            [-q[2],  q[1],  q[0]]
        ])
        F[:4, 4:] = -Xi * dt 
        
        # Predição da Covariância P
        # P = F * P * F^T + Q
        # A incerteza aumenta devido ao ruído do processo (Giroscópio).
        G = np.zeros((7, 6)) # Matriz de mapeamento de ruído
        G[:4, :3] = -Xi * dt
        G[4:, 3:] = np.eye(3) * dt
        
        self.P = F @ self.P @ F.T + (G @ self.Q_noise @ G.T)

        
        # ====================================================================
        # CORREÇÃO 1 - ACELERÔMETRO (Measurement Update)
        # Corrige Pitch e Roll usando o vetor gravidade.
        # ====================================================================
        qw, qx, qy, qz = self.x_state[:4]

        # Modelo de Medição h(x)
        # Estima onde o vetor gravidade [0, 0, 1] estaria no referencial do corpo
        # dada a orientação atual. Corresponde à última linha da DCM.
        h_accel = np.array([
            2*(qx*qz - qw*qy), 
            2*(qy*qz + qw*qx), 
            qw**2 - qx**2 - qy**2 + qz**2
        ])

        # Jacobiana H (Medição)
        # Derivada de h(x) em relação ao estado. Descreve como mudanças no 
        # quaternião afetam a leitura esperada da gravidade.
        H_accel = np.zeros((3, 7))
        H_accel[0,0]=-2*qy; H_accel[0,1]=2*qz; H_accel[0,2]=-2*qw; H_accel[0,3]=2*qx
        H_accel[1,0]=2*qx;  H_accel[1,1]=2*qw; H_accel[1,2]=2*qz;  H_accel[1,3]=2*qy
        H_accel[2,0]=2*qw;  H_accel[2,1]=-2*qx; H_accel[2,2]=-2*qy; H_accel[2,3]=2*qz

        # Inovação (Resíduo) y
        # Diferença entre a leitura real (z) e a esperada (h).
        y_a = z_accel - h_accel

        # Cálculo do Ganho de Kalman (K)
        # S = H*P*H^T + R (Covariância da inovação)
        # K = P*H^T * S^-1
        S_a = H_accel @ self.P @ H_accel.T + self.R_accel
        K_a = self.P @ H_accel.T @ np.linalg.inv(S_a)
        
        # Atualização do Estado e Covariância
        self.x_state += K_a @ y_a                 # Corrige estado
        self.P = (np.eye(7) - K_a @ H_accel) @ self.P # Reduz incerteza
        self.x_state[:4] = self._normalize_quaternion(self.x_state[:4])

        
        # ====================================================================
        # CORREÇÃO 2 - MAGNETÔMETRO (Measurement Update)
        # Corrige Yaw (Azimute) usando o vetor Norte.
        # ====================================================================
        q = self.x_state[:4] 
        R_mat = self._quaternion_to_rotation_matrix(q)
        
        # Predição da Medição h(x)
        # Rotaciona o vetor de referência (mag_ref) do mundo para o corpo.
        # h_mag = R_transposta * mag_ref
        h_mag = R_mat.T @ self.mag_ref 
        
        # Jacobiana H para Magnetômetro
        # Derivada da rotação do vetor de referência em relação ao quaternião.
        mx_ref, my_ref, mz_ref = self.mag_ref
        qw, qx, qy, qz = q
        H_mag = np.zeros((3, 7))
        H_mag_q = 2 * np.array([
            [-my_ref*qz + mz_ref*qy, mz_ref*qz + my_ref*qy, mz_ref*qw + mx_ref*qz - my_ref*qx, my_ref*qw - mx_ref*qy - mz_ref*qx],
            [mx_ref*qz + mz_ref*qy - my_ref*qw, mz_ref*qy + my_ref*qz, mx_ref*qw - my_ref*qx + mz_ref*qz, mx_ref*qy - my_ref*qw + mz_ref*qx],
            [-mx_ref*qy + my_ref*qx - mz_ref*qw, my_ref*qy - mx_ref*qz, my_ref*qw + mx_ref*qy - mz_ref*qx, mx_ref*qw - my_ref*qz + mz_ref*qy]
        ])
        H_mag[:, :4] = H_mag_q
        
        # Atualização Kalman (Mesma lógica do acelerômetro)
        y_m = z_mag - h_mag # Inovação
        S_m = H_mag @ self.P @ H_mag.T + self.R_mag # Covariância da Inovação
        K_m = self.P @ H_mag.T @ np.linalg.inv(S_m) # Ganho de Kalman
        
        correction = K_m @ y_m
        self.x_state += correction
        self.P = (np.eye(7) - K_m @ H_mag) @ self.P
        self.x_state[:4] = self._normalize_quaternion(self.x_state[:4])


        # --- Dead Reckoning (Posição) ---
        if self.enable_pos:
            # Pega aceleração bruta e divide pela calibração para ter unidades G
            accel_in_g = accel_raw / self.g_magnitude

            # Rotaciona o vetor de aceleração do referencial do Sensor para o Mundo
            R_curr = self._quaternion_to_rotation_matrix(self.x_state[:4])
            accel_world = R_curr @ accel_in_g

            # Subtrai a gravidade (1.0G no eixo Z) para sobrar apenas a aceleração linear (movimento)
            accel_world[2] -= 1.0
            
            # Filtro de Limiar (Deadband): Ignora ruídos pequenos
            linear_accel = np.zeros(3)
            if abs(accel_world[0]) > self.accel_threshold: linear_accel[0] = accel_world[0]
            if abs(accel_world[1]) > self.accel_threshold: linear_accel[1] = accel_world[1]
            if abs(accel_world[2]) > (self.accel_threshold * 1.5): linear_accel[2] = accel_world[2]
            
            # Integração: Aceleração (G) -> Aceleração (m/s²) -> Velocidade (m/s)
            self.vel += linear_accel * 9.81 * dt

            # Arrasto (Drag): Simula atrito para a velocidade não crescer infinitamente com erros
            self.vel[0] *= self.drag_xy
            self.vel[1] *= self.drag_xy
            self.vel[2] *= self.drag_z
            
            # ZUPT (Zero Velocity Update): Se a aceleração total é ~1G, assumimos que está parado
            raw_mag_accel = np.linalg.norm(accel_raw)
            if abs(raw_mag_accel - self.g_magnitude) < 0.05 * self.g_magnitude:
                self.vel *= 0.8

            # Integração: Velocidade -> Posição
            self.pos += self.vel * dt    
            
            # Retorno ao Centro: Traz o objeto de volta suavemente para a origem
            if self.return_center and np.linalg.norm(self.vel) < 0.05:
                self.pos *= self.center_speed
        else:
            self.pos = np.zeros(3)
            self.vel = np.zeros(3)

        # Debug
        if self.debug_mode and (current_time - self.last_debug_time > 0.5):
            self.last_debug_time = current_time
            
            est_yaw = self._get_yaw_from_quat(self.x_state[:4])
            raw_heading = math.degrees(math.atan2(z_mag[1], z_mag[0]))
            corr_mag = np.linalg.norm(correction[:4]) # Intensidade da correção magnética
            
            # Média do ganho de Kalman (Quanto maior, mais confia no sensor)
            gain_avg = np.mean(np.abs(K_m))
            
            print(f"\n--- EKF DEBUG ---")
            print(f"Yaw Est: {est_yaw:6.1f}° | Raw Mag: {raw_heading:6.1f}°")
            print(f"Resíduo Mag: [{y_m[0]:.2f}, {y_m[1]:.2f}, {y_m[2]:.2f}]")
            print(f"Ganho K (Mag): {gain_avg:.5f} | Correção: {corr_mag:.5f}")
            pos_str = f"({self.pos[0]:.2f}, {self.pos[1]:.2f}, {self.pos[2]:.2f})" if self.enable_pos else "OFF"
            print(f"Posição: {pos_str}")

         # Publica os dados
        q = self.x_state[:4]
        # Diagonal da matriz P representa a variância estimada de cada estado
        variances = np.diag(self.P)
        
        output = OrientationData(
            q[1], q[2], q[3], q[0], # qx, qy, qz, qw
            self.pos[0], self.pos[1], self.pos[2],
            *variances[:4] # Envia variâncias para visualização de incerteza
        )
        self.mediator.publish(self.output_topic, output)