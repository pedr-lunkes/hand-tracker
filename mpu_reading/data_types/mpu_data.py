"""
mpu_data.py

Defini uma classe simples, MpuData, que guarda os dados dos 9-eixos 
que são lidos pelo MPU9250 (acelerometro de 3 eixos, giroscópido de 3 eixos,
magnetometro de 3 eixos)
"""

class MpuData:
    def __init__(self, ax, ay, az, gx, gy, gz, mx, my, mz):
        """
        Inicializa a classe com os dados brutos do sensor

        Args:
            ax (float): Aceleração ao longo do eixo X (em g's)
            ay (float): Aceleração ao longo do eixo Y (em g's)
            az (float): Aceleração ao longo do eixo Z (em g's)
            gx (float): Leitura do giroscópio ao longo do eixo X (em graus/segundo)
            gy (float): Leitura do giroscópio ao longo do eixo Y (em graus/segundo)
            gz (float): Leitura do giroscópio ao longo do eixo Z (em graus/segundo)
            mx (float): Leitura do magnetômetro ao longo do eixo X (em microteslas)
            my (float): Leitura do magnetômetro ao longo do eixo Y (em microteslas)
            mz (float): Leitura do magnetômetro ao longo do eixo Z (em microteslas)
        """
        self.ax = ax
        self.ay = ay
        self.az = az
        self.gx = gx
        self.gy = gy
        self.gz = gz
        self.mx = mx
        self.my = my
        self.mz = mz
        

    def __str__(self):
        """
        Retorna uma representação em string do objeto MpuData para facilitar a impressão.
        """
        return (f"Accel (g): [x: {self.ax:+.4f}, y: {self.ay:+.4f}, z: {self.az:+.4f}] | "
            f"Gyro (dps): [x: {self.gx:+.4f}, y: {self.gy:+.4f}, z: {self.gz:+.4f}] | "
            f"Mag (µT): [x: {self.mx:+.4f}, y: {self.my:+.4f}, z: {self.mz:+.4f}]")