"""
gyro_calibration.py

Calcula a média de erro (Bias) do Giroscópio quando o sensor está parado.
Atualiza o config.yaml automaticamente.
"""

import time
import sys
import os
import yaml
import numpy as np
from mediator import Mediator
from data_types.mpu_data import MpuData
from handlers.serial_handler import MpuSerialHandler
from handlers.ble_handler import BleDataHandler

# --- CONFIG ---
DATA_SOURCE = "BLE"   

SERIAL_PORT = '/dev/ttyACM0'
BAUDRATE = 115200
BLE_DEVICE_NAME = "HandTracker-MPU" 

CONFIG_FILE = "config_real.yaml"
SAMPLES_TO_COLLECT = 2000 # ~20 segundos a 100Hz

class GyroCalibrator:
    def __init__(self, mediator):
        self.mediator = mediator
        self.mediator.subscribe("sensor", self.handle_data)
        self.raw_data = []
        self.collecting = False
        
    def handle_data(self, data: MpuData):
        if self.collecting:
            # Coleta gx, gy, gz
            self.raw_data.append([data.gx, data.gy, data.gz])

    def run(self):
        print("\n" + "="*50)
        print("   CALIBRAÇÃO DE BIAS DO GIROSCÓPIO   ")
        print("="*50)
        print("IMPORTANTE: O sensor deve ficar TOTALMENTE PARADO.")
        print("Não encoste na mesa. Não respire perto dele.")
        print("-" * 50)
        
        for i in range(5, 0, -1):
            print(f"Começando em {i}...", end='\r')
            time.sleep(1)
            
        print("\n>>> COLETANDO DADOS... MANTENHA PARADO! <<<")
        self.collecting = True
        self.raw_data = []
        
        while len(self.raw_data) < SAMPLES_TO_COLLECT:
            sys.stdout.write(f"\rAmostras: {len(self.raw_data)} / {SAMPLES_TO_COLLECT}")
            sys.stdout.flush()
            time.sleep(0.01)
            
        self.collecting = False
        print("\n\nCalculando média de erro...")
        self.calculate_and_save()

    def calculate_and_save(self):
        if not self.raw_data: return

        data = np.array(self.raw_data)
        
        # O Bias é simplesmente a média das leituras quando parado
        bias = np.mean(data, axis=0)
        
        # Calcula ruído (desvio padrão) apenas para curiosidade
        noise = np.std(data, axis=0)
        
        print("-" * 30)
        print(f"X Bias: {bias[0]:.4f} (Ruído: {noise[0]:.4f})")
        print(f"Y Bias: {bias[1]:.4f} (Ruído: {noise[1]:.4f})")
        print(f"Z Bias: {bias[2]:.4f} (Ruído: {noise[2]:.4f})")
        print("-" * 30)
        
        bias_list = [float(f"{x:.4f}") for x in bias]
        self.update_yaml(bias_list)

    def update_yaml(self, bias):
        if not os.path.exists(CONFIG_FILE):
            print(f"Erro: {CONFIG_FILE} não encontrado.")
            return

        print(f"\nSalvando em '{CONFIG_FILE}'...")
        try:
            with open(CONFIG_FILE, 'r') as f:
                config = yaml.safe_load(f)
            
            # Cria seção gyroscope se não existir
            if 'gyroscope' not in config:
                config['gyroscope'] = {}
            
            config['gyroscope']['bias'] = bias
            
            with open(CONFIG_FILE, 'w') as f:
                yaml.dump(config, f, default_flow_style=False, sort_keys=False)
            print("SUCESSO! Bias salvo.")
            
        except Exception as e:
            print(f"Erro ao salvar: {e}")

def main():
    mediator = Mediator()
    
    # Configura Handler (Copie do seu main.py se necessário)
    handler = None
    if DATA_SOURCE == "SERIAL":
        handler = MpuSerialHandler(SERIAL_PORT, BAUDRATE, mediator)
    elif DATA_SOURCE == "BLE":
        handler = BleDataHandler(BLE_DEVICE_NAME, mediator)
        pass
        
    if handler:
        handler.daemon = True
        handler.start()
        time.sleep(2) # Espera conectar
        
        calibrator = GyroCalibrator(mediator)
        calibrator.run()
        
        handler.stop()
        os._exit(0)

if __name__ == "__main__":
    main()