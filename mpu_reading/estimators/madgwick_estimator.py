"""
madgwick_estimator.py

Filtro Madgwick Original (9-DOF) com Compensação de Distorção Magnética.
Funde Giroscópio (Rotação), Acelerômetro (Gravidade) e Magnetômetro (Norte).
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
        
        # Carrega configurações
        print(f"Madgwick 9-DOF: Carregando '{config_path}'...")
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)
            
        self.input_topic = self.config['topics']['input_topic']
        self.output_topic = self.config['topics']['output_topic']
        self.mediator.subscribe(self.input_topic, self.handle_sensor_data)
        
        # Parâmetro Beta (Ganho do Filtro)
        # Beta alto (>0.1): Corrige drift rápido, mas deixa passar ruído (tremor).
        # Beta baixo (<0.03): Muito suave, mas demora a corrigir o Norte se houver distorção.
        self.beta = self.config['madgwick']['beta']
        
        # Estado Inicial (Quaternião Identidade)
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        
        # Calibrações (Carregadas do YAML gerado pelos scripts de calibração)
        # Magnetômetro (Hard/Soft Iron)
        self.mag_bias = np.array(self.config['magnetometer']['bias'])
        self.mag_scale = np.array(self.config['magnetometer']['scale'])
        
        # Giroscópio
        gyro_conf = self.config.get('gyroscope', {})
        self.gyro_bias = np.array(gyro_conf.get('bias', [0.0, 0.0, 0.0]))
        
        # --- Dead Reckoning (Posição) ---
        dr_conf = self.config['dead_reckoning']
        self.enable_pos = dr_conf.get('enable', False)
        self.accel_threshold = dr_conf['accel_threshold']
        self.calibration_frames = dr_conf['calibration_frames']
        self.drag_xy = dr_conf['drag']['xy']
        self.drag_z = dr_conf['drag']['z']
        self.return_center = dr_conf['return_to_center']
        self.center_speed = dr_conf.get('center_speed', 0.98)
        
        self.pos = np.array([0.0, 0.0, 0.0])
        self.vel = np.array([0.0, 0.0, 0.0])
        
        # Controle de tempo e calibração inicial G
        self.last_time = time.time()
        self.last_debug_time = 0
        self.calibration_samples = []
        self.g_magnitude = 9.81
        self.is_calibrating_g = True
        
        print(f"Madgwick 9-DOF Iniciado. Gyro Bias: {self.gyro_bias}")

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
        self.last_time = current_time
        if dt > 0.1: dt = 0.01

        # Correção de Bias do Giroscópio
        raw_gx = sensor_data.gx - self.gyro_bias[0]
        raw_gy = sensor_data.gy - self.gyro_bias[1]
        raw_gz = sensor_data.gz - self.gyro_bias[2]

        # Converte para Radianos (Unidade padrão da matemática de rotação)
        gx = math.radians(raw_gx)
        gy = math.radians(raw_gy)
        gz = math.radians(raw_gz)
        ax, ay, az = sensor_data.ax, sensor_data.ay, sensor_data.az
        
        # --- Rotina de Calibração da Gravidade ---
        if self.is_calibrating_g:
            norm_raw = math.sqrt(ax*ax + ay*ay + az*az)
            self.calibration_samples.append(norm_raw)
            if len(self.calibration_samples) > self.calibration_frames:
                avg = np.mean(self.calibration_samples)
                self.g_magnitude = avg if 0.5 < avg < 1.5 else 1.0
                self.is_calibrating_g = False
                print(f"Gravidade Calibrada: {self.g_magnitude:.3f}")
            return

        # Normalização do Acelerômetro
        # Para o Madgwick, não importa a FORÇA da gravidade, apenas a DIREÇÃO.
        # Normalizar transforma o vetor em tamanho 1, facilitando a trigonometria.
        norm_a = math.sqrt(ax*ax + ay*ay + az*az)
        if norm_a == 0: return 
        ax /= norm_a; ay /= norm_a; az /= norm_a

        # Magnetômetro (Calibração + Ajuste de Eixos)
        # Aplica Hard Iron (Bias) e Soft Iron (Scale)
        mx = (sensor_data.mx - self.mag_bias[0]) * self.mag_scale[0]
        my = (sensor_data.my - self.mag_bias[1]) * self.mag_scale[1]
        mz = (sensor_data.mz - self.mag_bias[2]) * self.mag_scale[2]
        
        # Mapeamento de Eixos: O MPU9250 geralmente tem o Mag rotacionado.
        # Padrão comum: Mag_X = MPU_Y, Mag_Y = MPU_X, Mag_Z = -MPU_Z
        mx_s, my_s, mz_s = my, mx, -mz 
        
        norm_m = math.sqrt(mx_s*mx_s + my_s*my_s + mz_s*mz_s)
        if norm_m == 0: return
        mx_s /= norm_m; my_s /= norm_m; mz_s /= norm_m

        # --- ALGORITMO MADGWICK 9-DOF ---
        qw, qx, qy, qz = self.q
        
        # Variáveis auxiliares
        _2qw = 2.0 * qw; _2qx = 2.0 * qx; _2qy = 2.0 * qy; _2qz = 2.0 * qz

        # Predição (Giroscópio)
        # Calcula a derivada do quaternião baseada na velocidade angular (w).
        # Fórmula: dq/dt = 0.5 * q * w
        q_dot_w = 0.5 * (-qx*gx - qy*gy - qz*gz)
        q_dot_x = 0.5 * ( qw*gx + qy*gz - qz*gy)
        q_dot_y = 0.5 * ( qw*gy - qx*gz + qz*gx)
        q_dot_z = 0.5 * ( qw*gz + qx*gy - qy*gx)

        # Correção Magnética Inteligente
        # 1. Rotaciona a leitura atual do magnetômetro para o referencial da Terra
        #    Isso nos diz onde o fluxo magnético está apontando AGORA, segundo nossa estimativa atual.
        #    h = q * m * q_conjugado
        hx = mx_s * (1 - 2*qy*qy - 2*qz*qz) + my_s * (2*qx*qy - 2*qw*qz) + mz_s * (2*qx*qz + 2*qw*qy)
        hy = mx_s * (2*qx*qy + 2*qw*qz) + my_s * (1 - 2*qx*qx - 2*qz*qz) + mz_s * (2*qy*qz - 2*qw*qx)
        hz = mx_s * (2*qx*qz - 2*qw*qy) + my_s * (2*qy*qz + 2*qw*qx) + mz_s * (1 - 2*qx*qx - 2*qy*qy)
        
        # 2. Normaliza o fluxo magnético da Terra (b)
        #    Madgwick assume que o vetor magnético verdadeiro da Terra
        #    tem componente Norte (X) e componente Vertical (Z), mas ZERO componente Leste (Y).
        #    bx = sqrt(hx^2 + hy^2) -> "Junta" todo o fluxo horizontal no eixo X.
        #    bz = hz -> Mantém o fluxo vertical original.
        bx = math.sqrt(hx*hx + hy*hy)
        bz = hz 
        
        # Variáveis auxiliares para o gradiente
        _2bx = 2.0 * bx
        _2bz = 2.0 * bz

        # Gradiente Descendente (Correção)
        # Calcula a direção do erro combinando Acelerômetro (Gravidade) e Magnetômetro (Norte)
        
        # 1: Erro do Acelerômetro (Gravidade estimada vs Real)
        # O objetivo é encontrar a direção do erro, e empurrar o quaternião para diminuir esse erro
        
        # 2: Erro do Magnetômetro (Fluxo estimado vs Real)
        # Tenta alinhar o vetor 'b' (calculado acima) com a leitura 'm' (sensor).
        
        s_w = -_2qy * (2.0 * (qx*qz - qw*qy) - ax) + _2qx * (2.0 * (qw*qx + qy*qz) - ay) - _2bz * qy * (_2bx * (0.5 - qy*qy - qz*qz) + _2bz * (qx*qz - qw*qy) - mx_s) + (-_2bx * qz + _2bz * qx) * (_2bx * (qx*qy - qw*qz) + _2bz * (qw*qx + qy*qz) - my_s) + _2bx * qy * (_2bx * (qx*qz + qw*qy) + _2bz * (0.5 - qx*qx - qy*qy) - mz_s)
        s_x = _2qz * (2.0 * (qx*qz - qw*qy) - ax) + _2qw * (2.0 * (qw*qx + qy*qz) - ay) - 4.0 * qx * (1 - 2.0 * (qx*qx + qy*qy) - az) + _2bz * qz * (_2bx * (0.5 - qy*qy - qz*qz) + _2bz * (qx*qz - qw*qy) - mx_s) + (_2bx * qy + _2bz * qw) * (_2bx * (qx*qy - qw*qz) + _2bz * (qw*qx + qy*qz) - my_s) + (_2bx * qz - _2bz * 4.0 * qx) * (_2bx * (qx*qz + qw*qy) + _2bz * (0.5 - qx*qx - qy*qy) - mz_s)
        s_y = -_2qw * (2.0 * (qx*qz - qw*qy) - ax) + _2qz * (2.0 * (qw*qx + qy*qz) - ay) - 4.0 * qy * (1 - 2.0 * (qx*qx + qy*qy) - az) + (-_2bx * 4.0 * qy - _2bz * 2.0 * qz) * (_2bx * (0.5 - qy*qy - qz*qz) + _2bz * (qx*qz - qw*qy) - mx_s) + (_2bx * qx + _2bz * qz) * (_2bx * (qx*qy - qw*qz) + _2bz * (qw*qx + qy*qz) - my_s) + (_2bx * qw - _2bz * 4.0 * qy) * (_2bx * (qx*qz + qw*qy) + _2bz * (0.5 - qx*qx - qy*qy) - mz_s)
        s_z = _2qx * (2.0 * (qx*qz - qw*qy) - ax) + _2qy * (2.0 * (qw*qx + qy*qz) - ay) + (-_2bx * 4.0 * qz + _2bz * 2.0 * qy) * (_2bx * (0.5 - qy*qy - qz*qz) + _2bz * (qx*qz - qw*qy) - mx_s) + (-_2bx * qw + _2bz * qx) * (_2bx * (qx*qy - qw*qz) + _2bz * (qw*qx + qy*qz) - my_s) + (_2bx * qx + _2bz * 4.0 * qz) * (_2bx * (qx*qz + qw*qy) + _2bz * (0.5 - qx*qx - qy*qy) - mz_s)
        
        # Normaliza o passo do gradiente
        norm_s = math.sqrt(s_w*s_w + s_x*s_x + s_y*s_y + s_z*s_z)
        if norm_s > 0:
            s_w /= norm_s; s_x /= norm_s; s_y /= norm_s; s_z /= norm_s

        # Fusão (Feedback)
        # Aplica a correção na predição do giroscópio
        q_dot_w -= self.beta * s_w
        q_dot_x -= self.beta * s_x
        q_dot_y -= self.beta * s_y
        q_dot_z -= self.beta * s_z

        # Integração e Normalização
        self.q[0] += q_dot_w * dt
        self.q[1] += q_dot_x * dt
        self.q[2] += q_dot_y * dt
        self.q[3] += q_dot_z * dt
        self.q /= np.linalg.norm(self.q)

        # --- DEAD RECKONING ---
        if self.enable_pos:
            # Pega aceleração bruta e divide pela calibração para ter unidades G
            accel_vec_raw = np.array([sensor_data.ax, sensor_data.ay, sensor_data.az]) / self.g_magnitude

            # Rotaciona o vetor de aceleração do referencial do Sensor para o Mundo
            R = self._quaternion_to_rotation_matrix(self.q)
            accel_world = R @ accel_vec_raw

            # Subtrai a gravidade (1.0G no eixo Z) para sobrar apenas a aceleração linear (movimento)
            accel_world[2] -= 1.0 # Remove 1G
            
            # Filtro de Limiar (Deadband): Ignora ruídos pequenos
            lin_accel = np.zeros(3)
            if abs(accel_world[0]) > self.accel_threshold: lin_accel[0] = accel_world[0]
            if abs(accel_world[1]) > self.accel_threshold: lin_accel[1] = accel_world[1]
            if abs(accel_world[2]) > (self.accel_threshold * 1.5): lin_accel[2] = accel_world[2]
            
            # Integração: Aceleração (G) -> Aceleração (m/s²) -> Velocidade (m/s)
            self.vel += lin_accel * 9.81 * dt

            # Arrasto (Drag): Simula atrito para a velocidade não crescer infinitamente com erros
            self.vel[0] *= self.drag_xy; self.vel[1] *= self.drag_xy; self.vel[2] *= self.drag_z
            
            # ZUPT (Zero Velocity Update): Se a aceleração total é ~1G, assumimos que está parado
            raw_mag = np.linalg.norm([sensor_data.ax, sensor_data.ay, sensor_data.az])
            if abs(raw_mag - self.g_magnitude) < 0.05 * self.g_magnitude: self.vel *= 0.8 # ZUPT

            # Integração: Velocidade -> Posição
            self.pos += self.vel * dt

            # Retorno ao Centro: Traz o objeto de volta suavemente para a origem
            if self.return_center and np.linalg.norm(self.vel) < 0.05: self.pos *= self.center_speed
        else:
            self.pos = np.zeros(3)
            self.vel = np.zeros(3)

        # Debug
        if self.debug_mode and (current_time - self.last_debug_time > 0.5):
            self.last_debug_time = current_time
            yaw = math.degrees(math.atan2(2.0*(self.q[1]*self.q[2] + self.q[0]*self.q[3]), 
                                          self.q[0]**2 + self.q[1]**2 - self.q[2]**2 - self.q[3]**2))
            mag_heading = math.degrees(math.atan2(mx_s, my_s))
            print(f"Madgwick 9-DOF: Yaw={yaw:.1f}° | MagRaw={mag_heading:.1f}° | Pos={self.pos[0]:.2f}")

        # Publica os dados
        output = OrientationData(
            self.q[1], self.q[2], self.q[3], self.q[0], 
            self.pos[0], self.pos[1], self.pos[2],
            0, 0, 0, 0       
        )
        self.mediator.publish(self.output_topic, output)