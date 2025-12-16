"""
calibration.py

Script utilitário para calibrar o magnetômetro (MPU9250).
Ele coleta dados brutos, calcula Hard Iron (Bias) e Soft Iron (Scale),
e atualiza automaticamente o arquivo 'config.yaml'.
"""

import time
import sys
import os
import yaml
import numpy as np
import threading

# Imports do seu projeto
from mediator import Mediator
from data_types.mpu_data import MpuData
from handlers.serial_handler import MpuSerialHandler
from handlers.ble_handler import BleDataHandler
# Se precisar simular: from handlers.keyboard_handler import KeyboardHandler

# --- CONFIGURAÇÃO DA CONEXÃO ---
# Ajuste conforme seu setup atual
DATA_SOURCE = "BLE" # ou "BLE"

SERIAL_PORT = '/dev/ttyACM0' # Ajuste para sua porta
BAUDRATE = 115200
BLE_DEVICE_NAME = "HandTracker-MPU"
CONFIG_FILE = "config_real.yaml"

CALIBRATION_DURATION = 45 # Segundos

class MagnetometerCalibrator:
    def __init__(self, mediator):
        self.mediator = mediator
        self.mediator.subscribe("sensor", self.handle_data)
        
        self.raw_data = []
        self.collecting = False
        self.sample_count = 0
        
    def handle_data(self, data: MpuData):
        """Callback que recebe dados do sensor."""
        if self.collecting:
            # Armazena apenas dados do magnetômetro (mx, my, mz)
            # Nota: Armazenamos os dados BRUTOS que chegam do handler.
            # O handler geralmente já converteu bytes para float, mas não aplicou bias ainda.
            if data.mx == 0 and data.my == 0 and data.mz == 0:
                return # Ignora zeros absolutos (erro de leitura)
                
            self.raw_data.append([data.mx, data.my, data.mz])
            self.sample_count += 1

    def run(self):
        print("\n" + "="*50)
        print("   CALIBRADOR DE MAGNETÔMETRO (HARD/SOFT IRON)   ")
        print("="*50)
        print(f"Fonte de Dados: {DATA_SOURCE}")
        print("Prepara-se para girar o sensor em formato de '8'.")
        print("Tente cobrir todos os ângulos (cima, baixo, lados).")
        print(f"A coleta durará {CALIBRATION_DURATION} segundos.")
        print("-" * 50)
        
        for i in range(5, 0, -1):
            print(f"Iniciando em {i}...", end='\r')
            time.sleep(1)
        
        print("\n>>> COLETANDO DADOS... GIRE O SENSOR AGORA! <<<")
        self.collecting = True
        self.raw_data = []
        
        start_time = time.time()
        try:
            while time.time() - start_time < CALIBRATION_DURATION:
                elapsed = int(time.time() - start_time)
                remaining = CALIBRATION_DURATION - elapsed
                sys.stdout.write(f"\rTempo: {elapsed}s / {CALIBRATION_DURATION}s | Amostras: {self.sample_count}")
                sys.stdout.flush()
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\n\nColeta interrompida pelo usuário.")
        
        self.collecting = False
        print("\n\nCalculando calibração...")
        self.calculate_and_save()

    def calculate_and_save(self):
        if not self.raw_data:
            print("Erro: Nenhum dado coletado.")
            return

        data = np.array(self.raw_data)
        
        # Encontra Mínimos e Máximos para cada eixo
        # min_vals = [min_x, min_y, min_z]
        min_vals = np.min(data, axis=0)
        max_vals = np.max(data, axis=0)
        
        print("-" * 30)
        print(f"X: Min={min_vals[0]:.2f}, Max={max_vals[0]:.2f}")
        print(f"Y: Min={min_vals[1]:.2f}, Max={max_vals[1]:.2f}")
        print(f"Z: Min={min_vals[2]:.2f}, Max={max_vals[2]:.2f}")
        print("-" * 30)

        # --- CÁLCULO HARD IRON (BIAS) ---
        # Média entre extremos: (Max + Min) / 2
        bias = (max_vals + min_vals) / 2.0

        # --- CÁLCULO SOFT IRON (SCALE) ---
        # Distância entre extremos (corda): (Max - Min) / 2
        chord_x = (max_vals[0] - min_vals[0]) / 2.0
        chord_y = (max_vals[1] - min_vals[1]) / 2.0
        chord_z = (max_vals[2] - min_vals[2]) / 2.0

        avg_chord = (chord_x + chord_y + chord_z) / 3.0

        scale_x = avg_chord / chord_x if chord_x != 0 else 1.0
        scale_y = avg_chord / chord_y if chord_y != 0 else 1.0
        scale_z = avg_chord / chord_z if chord_z != 0 else 1.0

        # Formata para lista python simples
        bias_list = [float(f"{x:.2f}") for x in bias]
        scale_list = [float(f"{x:.3f}") for x in [scale_x, scale_y, scale_z]]

        print("\nRESULTADOS:")
        print(f"Bias Calculado:  {bias_list}")
        print(f"Scale Calculado: {scale_list}")

        self.update_yaml(bias_list, scale_list)

    def update_yaml(self, bias, scale):
        if not os.path.exists(CONFIG_FILE):
            print(f"Erro: Arquivo '{CONFIG_FILE}' não encontrado.")
            return

        print(f"\nAtualizando '{CONFIG_FILE}'...")
        
        try:
            # Carrega o YAML existente preservando estrutura (tanto quanto possível com PyYAML)
            with open(CONFIG_FILE, 'r') as f:
                config = yaml.safe_load(f)
            
            # Atualiza os valores
            if 'magnetometer' not in config:
                config['magnetometer'] = {}
            
            config['magnetometer']['bias'] = bias
            config['magnetometer']['scale'] = scale
            
            # Salva de volta
            with open(CONFIG_FILE, 'w') as f:
                yaml.dump(config, f, default_flow_style=False, sort_keys=False)
                
            print("SUCESSO! Configuração salva.")
            print("Agora você pode rodar o main.py com a nova calibração.")
            
        except Exception as e:
            print(f"Erro ao salvar YAML: {e}")

def main():
    mediator = Mediator()
    
    # Inicializa Handler (Cópia da lógica do main.py)
    handler = None
    if DATA_SOURCE == "BLE":
        print(f"Conectando BLE: {BLE_DEVICE_NAME}...")
        handler = BleDataHandler(BLE_DEVICE_NAME, mediator)
    elif DATA_SOURCE == "SERIAL":
        print(f"Conectando Serial: {SERIAL_PORT}...")
        handler = MpuSerialHandler(SERIAL_PORT, BAUDRATE, mediator)
    else:
        print("Data Source desconhecido.")
        return

    handler.daemon = True # Importante para fechar com Ctrl+C
    handler.start()

    # Inicia Calibrador
    calibrator = MagnetometerCalibrator(mediator)
    
    try:
        # Aguarda conexão estabilizar
        time.sleep(2) 
        calibrator.run()
    except Exception as e:
        print(f"Erro: {e}")
    finally:
        print("\nEncerrando conexão...")
        if handler: handler.stop()
        os._exit(0)

if __name__ == "__main__":
    main()