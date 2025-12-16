"""
orientation_data.py

Define a estrutura de dados padrão para comunicação entre os Estimadores
(EKF/Madgwick) e os Consumidores (Visualizadores/Loggers).
"""

class OrientationData:
    """
    Classe DTO (Data Transfer Object) para o estado estimado do sistema.
    
    Esta classe encapsula três componentes fundamentais da navegação inercial:
    1. Atitude/Orientação (Quaterniões).
    2. Posição Relativa (Dead Reckoning).
    3. Incerteza Estatística (Variância da Covariância P).
    """
    def __init__(self, qx, qy, qz, qw, x, y, z, var_qx, var_qy, var_qz, var_qw):
        # --- 1. Orientação (Quaterniões) ---
        # Representação de rotação no espaço 3D usando números hipercomplexos.
        # Preferível aos Ângulos de Euler por evitar o problema de 'Gimbal Lock'
        # e ser computacionalmente mais eficiente para interpolação (SLERP).
        # q = [qx, qy, qz, qw], onde qw é a parte escalar e (qx,qy,qz) a vetorial.
        self.qx, self.qy, self.qz, self.qw = qx, qy, qz, qw
        
        # --- 2. Posição (Dead Reckoning) ---
        # Coordenadas Cartesianas (X, Y, Z) em metros (relativo ao ponto de partida).
        # Resultado da dupla integração da aceleração linear (a -> v -> p).
        # Nota: Sujeito a deriva (drift) acumulativa ao longo do tempo.
        self.x, self.y, self.z = x, y, z
        
        # --- 3. Incerteza Estatística (Apenas EKF) ---
        # Representa a diagonal principal da Matriz de Covariância (P) do Filtro de Kalman.
        # Variância (sigma^2) indica o grau de confiança do filtro na estimativa atual.
        # - Valor Alto: Alta incerteza (O filtro confia pouco no cálculo).
        # - Valor Baixo: Alta precisão (O filtro convergiu).
        self.var_qx, self.var_qy, self.var_qz, self.var_qw = var_qx, var_qy, var_qz, var_qw