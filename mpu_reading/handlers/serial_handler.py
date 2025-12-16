"""
serial_handler.py

Conecta-se a uma porta serial (USB) e publica os dados do MPU
no tópico "sensor" do mediator.
"""

import serial
import time
import threading
from data_types.mpu_data import MpuData
from mediator import Mediator

class MpuSerialHandler:
    """
    Gerencia a conexão com a porta serial e a recepção de dados em uma thread.
    Publica MpuData no mediator.
    """

    def __init__(self, port, baudrate, mediator: Mediator):
        self.port = port
        self.baudrate = baudrate
        self.mediator = mediator
        self.serial_port = None
        self._running = False
        self.thread = None

    def start(self):
        """Abre a porta serial e inicia a thread de leitura."""
        try:
            self.serial_port = serial.Serial(self.port, self.baudrate, timeout=1.0)
            print(f"Serial port {self.port} opened.")
            time.sleep(2.0) 
            self.serial_port.flushInput()
            
            self._running = True
            self.thread = threading.Thread(target=self._run_loop, daemon=True)
            self.thread.start()
            
        except serial.SerialException as e:
            print(f"Failed to open serial port {self.port}: {e}")
            self.serial_port = None

    def stop(self):
        """Encerra a thread de leitura e fecha a porta serial."""
        print("Stopping Serial handler...")
        self._running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=2.0)
            
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            print("Serial port closed.")
        print("Serial handler stopped.")

    def is_ready(self):
        """Verifica se a porta serial está aberta."""
        return self.serial_port is not None and self.serial_port.is_open

    def _run_loop(self):
        """
        Lê os dados da porta serial e os publica.
        Este método é executado em sua própria thread.
        """
        print("Serial read thread started.")
        while self._running and self.is_ready():
            try:
                line = self.serial_port.readline().decode('utf-8').strip()
                
                if not line:
                    continue # Timeout

                parts = line.split(' ')
                if len(parts) == 9:
                    ax, ay, az, gx, gy, gz, mx, my, mz = map(float, parts)
                    mpu_data = MpuData(ax, ay, az, gx, gy, gz, mx, my, mz)
                    # Publica os dados no mediator
                    self.mediator.publish("sensor", mpu_data)
                
            except serial.SerialException:
                print("Serial port disconnected. Stopping thread.")
                self._running = False
        
        print("Serial read thread finished.")
