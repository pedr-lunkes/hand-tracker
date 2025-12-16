"""
madwick6d_estimator.py

Estimador 6-DOF (Giroscópio + Acelerômetro).
Estabiliza Pitch/Roll usando a gravidade, mas o Yaw possui drift.
"""

import time
import math
import numpy as np
import yaml

from data_types.orientation_data import OrientationData
from data_types.mpu_data import MpuData
from mediator import Mediator

class Madgwick6DEstimator:
    def __init__(self, mediator: Mediator, config_path="config.yaml"):
        self.mediator = mediator
        
        # Carrega configurações (YAML)
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)
            
        self.input_topic = self.config['topics']['input_topic']
        self.output_topic = self.config['topics']['output_topic']
        self.mediator.subscribe(self.input_topic, self.handle_sensor_data)
        
        # Parâmetro Beta (Ganho do Filtro)
        # Beta alto (>0.1): Corrige drift rápido, mas deixa passar ruído (tremor).
        # Beta baixo (<0.03): Muito suave, mas demora a corrigir o Norte se houver distorção.
        self.beta = self.config.get('imu_6dof', {}).get('beta', 0.03)
        
        # Estado Inicial: Quaternião Identidade (w=1, x=0, y=0, z=0) -> Rotação Zero.
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        
        # Configurações de Dead Reckoning (estimativa de posição)
        dr_conf = self.config.get('dead_reckoning', {})
        self.enable_pos = dr_conf.get('enable', False)
        self.accel_threshold = dr_conf.get('accel_threshold', 0.1)
        self.drag_xy = dr_conf.get('drag', {}).get('xy', 0.9)
        self.drag_z = dr_conf.get('drag', {}).get('z', 0.9)
        self.return_center = dr_conf.get('return_to_center', True)
        self.center_speed = dr_conf.get('center_speed', 0.98)
        self.calibration_frames = dr_conf.get('calibration_frames', 100)
        self.debug_mode = dr_conf.get('debug', False)

        # Bias do Giroscópio (Calibração estática)
        gyro_conf = self.config.get('gyroscope', {})
        self.gyro_bias = np.array(gyro_conf.get('bias', [0.0, 0.0, 0.0]))

        # Variáveis de Estado (Posição/Velocidade)
        self.pos = np.array([0.0, 0.0, 0.0])
        self.vel = np.array([0.0, 0.0, 0.0])
        
        self.last_time = time.time()
        self.last_debug_time = 0
        
        # Calibração de gravidade
        self.calibration_samples = []
        self.g_magnitude = 9.81
        self.is_calibrating_g = True
        
        print(f"IMU Estimator 6-DOF Iniciado (Beta: {self.beta}).")

    def _quaternion_to_rotation_matrix(self, q):
        # Converte Quaternião para Matriz de Rotação (DCM)
        # Necessário para projetar a aceleração do corpo para o mundo real.
        qw, qx, qy, qz = q
        return np.array([
            [1 - 2*qy**2 - 2*qz**2, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
            [2*qx*qy + 2*qz*qw, 1 - 2*qx**2 - 2*qz**2, 2*qy*qz - 2*qx*qw],
            [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx**2 - 2*qy**2]
        ])

    def handle_sensor_data(self, sensor_data: MpuData):
        if not sensor_data: return

        # Delta Tempo (dt): O tempo passou desde a última leitura, utilizado na integração.
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
                print(f"IMU: Gravidade Calibrada: {self.g_magnitude:.3f}")
            return

        # Normalização do Acelerômetro
        # Para o Madgwick, não importa a FORÇA da gravidade, apenas a DIREÇÃO.
        # Normalizar transforma o vetor em tamanho 1, facilitando a trigonometria.
        norm_a = math.sqrt(ax*ax + ay*ay + az*az)
        if norm_a == 0: return
        ax /= norm_a; ay /= norm_a; az /= norm_a


        # --- MADGWICK 6-DOF ---
        qw, qx, qy, qz = self.q

        # Variáveis auxiliares
        _2qw = 2.0 * qw
        _2qx = 2.0 * qx
        _2qy = 2.0 * qy
        _2qz = 2.0 * qz

        # Predição pelo Giroscópio (Dead Reckoning Angular)
        # Calcula a derivada do quaternião baseada na velocidade angular (w).
        # Fórmula: dq/dt = 0.5 * q * w
        q_dot_w = 0.5 * (-qx*gx - qy*gy - qz*gz)
        q_dot_x = 0.5 * ( qw*gx + qy*gz - qz*gy)
        q_dot_y = 0.5 * ( qw*gy - qx*gz + qz*gx)
        q_dot_z = 0.5 * ( qw*gz + qx*gy - qy*gx)

        
        # Correção pelo Acelerômetro (Gradiente Descendente)
        # O objetivo aqui é encontrar a direção do erro.
        
        # 1. Definimos a Função Objetivo (f): A diferença entre a gravidade estimada e a medida.
        #    Sabemos que no mundo real a gravidade é [0, 0, 1].
        #    Rotacionamos [0,0,1] pelo nosso quaternião atual para ver onde o sensor "acha" que a gravidade está.
        #    Subtraímos isso da leitura do acelerômetro (ax, ay, az).
        
        #    f = [gravidade_estimada_x - ax]
        #        [gravidade_estimada_y - ay]
        #        [gravidade_estimada_z - az]

        # Componentes do vetor de erro (f)
        f_0 = 2.0*(qx*qz - qw*qy) - ax
        f_1 = 2.0*(qw*qx + qy*qz) - ay
        f_2 = 2.0*(0.5 - qx*qx - qy*qy) - az
        

        # 2. Calculamos o Gradiente (J_t * f):
        #    A Jacobiana (J) relaciona como uma mudança no quaternião afeta a gravidade estimada.
        #    Multiplicando J transposta pelo erro (f), obtemos a direção na qual devemos
        #    "empurrar" o quaternião para reduzir o erro.

        # Transposta da Jacobiana J_g
        # J_t = [-2qy,  2qy,  0  ]
        #       [ 2qz,  2qw, -4qx]
        #       [-2qw,  2qz, -4qy]
        #       [ 2qx,  2qy,  0  ]
        
        s_w = -_2qy * f_0 + _2qx * f_1
        s_x =  _2qz * f_0 + _2qw * f_1 - 4.0 * qx * f_2
        s_y = -_2qw * f_0 + _2qz * f_1 - 4.0 * qy * f_2
        s_z =  _2qx * f_0 + _2qy * f_1
        
        # Normaliza o gradiente
        norm_s = math.sqrt(s_w*s_w + s_x*s_x + s_y*s_y + s_z*s_z)
        if norm_s > 0:
            s_w /= norm_s; s_x /= norm_s; s_y /= norm_s; s_z /= norm_s

        # Fusão (Feedback)
        # Ajustamos a taxa de variação do quaternião (q_dot) subtraindo o erro encontrado.
        # Beta controla a força dessa correção.
        q_dot_w -= self.beta * s_w
        q_dot_x -= self.beta * s_x
        q_dot_y -= self.beta * s_y
        q_dot_z -= self.beta * s_z

        # Integração Numérica
        # Novo Quaternião = Antigo + (Taxa de Variação * Tempo)
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
            if abs(raw_mag - self.g_magnitude) < 0.05 * self.g_magnitude: self.vel *= 0.8

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
            pitch = math.degrees(math.asin(2.0 * (self.q[0]*self.q[2] - self.q[3]*self.q[1])))
            print(f"IMU 6-DOF: Yaw(Drift)={yaw:.1f}° | Pitch(Stable)={pitch:.1f}°")

        # Publica os dados
        output = OrientationData(
            self.q[1], self.q[2], self.q[3], self.q[0], 
            self.pos[0], self.pos[1], self.pos[2],
            0, 0, 0, 0       
        )
        self.mediator.publish(self.output_topic, output)