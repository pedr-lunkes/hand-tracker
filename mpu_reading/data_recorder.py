import time
import csv
import os
import threading
from datetime import datetime
from data_types.orientation_data import OrientationData
from mediator import Mediator

class Recorder:
    def __init__(self, mediator: Mediator, topic_name="orientation"):
        self.mediator = mediator
        self.topic_name = topic_name
        self.output_dir = "../training_data"
        self.is_recording = False
        self.current_label = "unknown"
        self.buffer = []
        self.running = True 

        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)

        self.mediator.subscribe(self.topic_name, self.handle_data)

    def handle_data(self, data: OrientationData):
        if self.is_recording:
            # Salva timestamp, label, quaternion(4) e posição(3)
            row = [time.time(), self.current_label, 
                   data.qx, data.qy, data.qz, data.qw,
                   data.x, data.y, data.z]
            self.buffer.append(row)

    def save(self):
        if not self.buffer: return
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{self.current_label}_{timestamp_str}.csv"
        path = os.path.join(self.output_dir, filename)
        
        with open(path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["ts", "label", "qx", "qy", "qz", "qw", "px", "py", "pz"])
            writer.writerows(self.buffer)
        print(f" -> Salvo: {path} ({len(self.buffer)} frames)")
        self.buffer = []

    def run_console(self):
        """Loop que roda na thread secundária ouvindo o teclado."""
        print("\n--- MODO GRAVAÇÃO VIA TERMINAL ---")
        print("Digite o LABEL e dê [ENTER] para começar.")
        print("Dê [ENTER] novamente para parar.")
        print("Digite 'SAIR' para encerrar o script.\n")

        while self.running:
            try:
                # 1. Aguarda Label para começar
                label = input(">> Digite o Label (ou ENTER p/ pular): ").strip()
                if label.upper() == 'SAIR': break
                if not label: continue

                # 2. Inicia Gravação
                self.current_label = label
                self.is_recording = True
                print(f"  [GRAVANDO '{label}'] ... (Pressione ENTER para parar)")
                
                # 3. Bloqueia esperando o ENTER de parada
                input() 
                
                # 4. Para e Salva
                self.is_recording = False
                self.save()
                
            except EOFError:
                break
        print("Recorder: Thread de console finalizada.")

    def stop(self):
        self.running = False