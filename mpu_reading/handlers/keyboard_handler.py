"""
keyboard_handler.py
Simula um sensor MPU.
Recebe comandos de controle via Mediator ("sim_control") e gera dados de sensor ("sensor").
"""

import time
import threading
import numpy as np
from data_types.mpu_data import MpuData
from mediator import Mediator

class KeyboardHandler:
    def __init__(self, mediator: Mediator):
        self.mediator = mediator
        # Inscreve-se para receber comandos de controle vindos do visualizador
        self.mediator.subscribe("sim_control", self.update_control)
        
        self._running = False
        self.thread = None
        
        # --- Simulação do Estado Físico ---
        self.gt_quaternion = np.array([1.0, 0.0, 0.0, 0.0]) 
        self.GRAVITY_WORLD = np.array([0.0, 0.0, 1.0]) 
        self.MAG_WORLD = np.array([1.0, 0.0, -0.5])
        self.MAG_WORLD /= np.linalg.norm(self.MAG_WORLD)

        # Inputs Atuais (recebidos do visualizador)
        self.input_accel = np.array([0.0, 0.0, 0.0])
        self.input_gyro = np.array([0.0, 0.0, 0.0])
        self.input_lock = threading.Lock()

    def start(self):
        """Inicia a thread de simulação física."""
        print("Keyboard Simulation Handler Started (Controlled via Pygame Window)")
        self._running = True
        self.thread = threading.Thread(target=self._run_loop, daemon=True)
        self.thread.start()

    def stop(self):
        self._running = False
        if self.thread and self.thread.is_alive():
            self.thread.join()
        print("Keyboard Handler stopped.")

    def update_control(self, data):
        """Callback: Recebe dicionário {'accel': [...], 'gyro': [...]}"""
        with self.input_lock:
            self.input_accel = np.array(data['accel'])
            self.input_gyro = np.array(data['gyro'])

    def _rotate_vector_inverse(self, v, q):
        """Rotaciona do MUNDO para o SENSOR (inverso)."""
        # q conjugado [w, -x, -y, -z]
        q_inv = np.array([q[0], -q[1], -q[2], -q[3]])
        
        w, x, y, z = q_inv
        q_vec = np.array([x, y, z])
        t = 2.0 * np.cross(q_vec, v)
        return v + w * t + np.cross(q_vec, t)

    def _integrate_gyro(self, dt):
        """Atualiza o 'Ground Truth' quaternion."""
        with self.input_lock:
            gx, gy, gz = self.input_gyro
        
        if np.linalg.norm([gx, gy, gz]) < 0.001: return

        w, x, y, z = self.gt_quaternion
        q_dot_w = 0.5 * (-x*gx - y*gy - z*gz)
        q_dot_x = 0.5 * ( w*gx + y*gz - z*gy)
        q_dot_y = 0.5 * ( w*gy - x*gz + z*gx)
        q_dot_z = 0.5 * ( w*gz + x*gy - y*gx)
        
        self.gt_quaternion += np.array([q_dot_w, q_dot_x, q_dot_y, q_dot_z]) * dt
        self.gt_quaternion /= np.linalg.norm(self.gt_quaternion)

    def _run_loop(self):
        Hz = 100.0
        dt = 1.0 / Hz
        
        while self._running:
            start_time = time.time()
            
            # 1. Física: Integra Giroscópio
            self._integrate_gyro(dt)
            
            # 2. Física: Calcula vetores relativos ao sensor
            with self.input_lock:
                current_accel_input = self.input_accel.copy()
                current_gyro_input = self.input_gyro.copy()

            # Acelerômetro lê (Gravidade + Aceleração Linear) no referencial do sensor
            total_force = current_accel_input + self.GRAVITY_WORLD
            accel_reading = self._rotate_vector_inverse(total_force, self.gt_quaternion)
            
            mag_reading = self._rotate_vector_inverse(self.MAG_WORLD, self.gt_quaternion)
            gyro_reading = np.degrees(current_gyro_input) 
            
            # 3. Ruído
            accel_reading += np.random.normal(0, 0.02, 3)
            mag_reading += np.random.normal(0, 0.05, 3)
            
            # 4. Publicar
            data = MpuData(
                accel_reading[0], accel_reading[1], accel_reading[2],
                gyro_reading[0], gyro_reading[1], gyro_reading[2],
                mag_reading[0], mag_reading[1], mag_reading[2]
            )
            self.mediator.publish("sensor", data)
            
            elapsed = time.time() - start_time
            if dt - elapsed > 0:
                time.sleep(dt - elapsed)