"""
main.py

Ponto de entrada principal para o aplicativo de rastreamento de orientação.
"""

import threading
import sys
import os
from mediator import Mediator

# --- Estimadores ---
from estimators.ekf_estimator import EkfEstimator
from estimators.madgwick_estimator import MadgwickEstimator
from estimators.madwick6d_estimator import Madgwick6DEstimator
from estimators.kalman_estimator import KalmanEstimator

from data_visulalization.visualizer_3d import CubeVisualizer
from data_visulalization.ekf_plotter import GaussianPlotter
from data_recorder import Recorder

# --- Data handles ---
from handlers.serial_handler import MpuSerialHandler
from handlers.ble_handler import BleDataHandler
from handlers.keyboard_handler import KeyboardHandler


# --- Configuração  ---
# Opções: "BLE", "SERIAL", "KEYBOARD"
DATA_SOURCE = "BLE"

# Configurações BLE
BLE_DEVICE_NAME = "HandTracker-MPU" 

# Configurações Serial
SERIAL_PORT = '/dev/ttyACM0' 
BAUDRATE = 115200
# ---------------------

def main():
    # Cria um mediador que funciona como um servidor para tópicos e assinantes
    # permite uma comunicação mais fácil entre as classes
    mediator = Mediator()
    
    # ---- Manipuladores ----
    # Inicialize o manipulador de dados escolhido :D -> de onde os dados virão?

    handler = None
    # Há suporte para Bluetooth Low Energy, Serial e um Teclado para simulação
    if DATA_SOURCE == "BLE":
        print(f"Using BLE data source. Connecting to '{BLE_DEVICE_NAME}'...")
        handler = BleDataHandler(BLE_DEVICE_NAME, mediator)
    elif DATA_SOURCE == "SERIAL":
        print(f"Using Serial data source on {SERIAL_PORT}...")
        handler = MpuSerialHandler(SERIAL_PORT, BAUDRATE, mediator)
    elif DATA_SOURCE == "KEYBOARD":
        # O modo teclado é apenas para simulação (ele possui saídas perfeitas e sem ruído
        # no magnetômetro). Deve-se usar um arquivo yaml correspondente D:
        print(f"Using Keyboard Simulation Mode...")
        handler = KeyboardHandler(mediator)
    else:
        print(f"Error: Unknown DATA_SOURCE '{DATA_SOURCE}'")
        sys.exit(1)

    # Inicia a thread do manipulador (ele irá se conectar e começar a publicar no tópico "sensor")
    handler.start()


    # ---- Definição do Estimador ----
    # Inicializa o estimador
    # É aqui que todos os cálculos para estimativa de orientação e posição serão feitos
    # Você deve criar um arquivo .yaml com os parâmetros adequados para o seu sensor, 
    # a fim de obter os melhores resultados :D
    # Os resultados são publicados no tópico "orientation"
    estimator = Madgwick6DEstimator(mediator, config_path="config_real.yaml")


    # ---- Feedback visual ----
    # Este é um plotter que mostra a incerteza nos cálculos de orientação
    # Descomente se quiser ver o Filtro de Kalman em ação >:D
    # plotter = GaussianPlotter(mediator)
    # plotter_thread = threading.Thread(target=plotter.run, daemon=True)
    # plotter_thread.start()

    # Esta é uma simulação visual de um cubo que reflete a orientação e posição calculadas
    visualizer = CubeVisualizer(mediator)

    recorder = Recorder(mediator, topic_name="orientation")
    recorder_thread = threading.Thread(target=recorder.run_console, daemon=True)
    recorder_thread.start()

    print("Iniciando GUI de Coleta...")
    

    print("Starting simulation... (Close plot window to exit)")
    try:
        # O visualizador é executado na thread principal, então se você fechá-lo o programa será encerrado
        visualizer.run()
    except KeyboardInterrupt:
        print("\nCaught Ctrl+C, stopping...")
    except Exception as e:
        print(f"Plotter closed due to an error: {e}")
    finally:
        # Pare o manipulador de dados
        handler.stop()
        
        # Para o gravador
        recorder.stop()
        
        print("Application shut down.")
        os._exit(0)


if __name__ == "__main__":
    main()