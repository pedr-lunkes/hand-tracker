"""
visualizer_3d.py

Visualizador 3D utilizando Pygame e OpenGL.
Responsável por desenhar o cubo e os eixos baseados nos dados de orientação
recebidos do Mediator. Também captura inputs de teclado para simulação.
"""

import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import threading
from mediator import Mediator

class CubeVisualizer:
    """
    Assina o tópico "orientation" do Mediator e renderiza uma cena 3D.
    Funciona no thread principal (necessário para o Pygame/OpenGL).
    """
    def __init__(self, mediator: Mediator):
        self.mediator = mediator
        # Se inscreve para receber dados do EKF ou Madgwick
        self.mediator.subscribe("orientation", self.update_orientation)
        
        # Estado interno para renderização
        # Quaternion [qx, qy, qz, qw] - Inicializado como Identidade (sem rotação)
        self.orientation = [0.0, 0.0, 0.0, 1.0]
        # Posição [x, y, z] - Inicializada na origem
        self.position = [0.0, 0.0, 0.0]
        
        # Lock para evitar que o visualizador leia dados enquanto o EKF está escrevendo
        self.data_lock = threading.Lock()
        
        self.width = 800
        self.height = 600
        
        # --- Definição da Geometria do Cubo ---
        # 8 Vértices (X, Y, Z)
        self.vertices = (
            (1, -1, -1), (1, 1, -1), (-1, 1, -1), (-1, -1, -1),
            (1, -1, 1), (1, 1, 1), (-1, -1, 1), (-1, 1, 1)
        )
        # Arestas (conecções entre vértices)
        self.edges = (
            (0,1), (0,3), (0,4), (2,1), (2,3), (2,7),
            (6,3), (6,4), (6,7), (5,1), (5,4), (5,7)
        )
        # Cores para cada face
        self.colors = (
            (1,0,0), (0,1,0), (0,0,1), (1,1,0), (0,1,1), (1,0,1)
        )
        # Faces (Quad) definidas pelos índices dos vértices
        self.surfaces = (
            (0,1,2,3), (3,2,7,6), (6,7,5,4),
            (4,5,1,0), (1,5,7,2), (4,0,3,6)
        )

    def update_orientation(self, data):
        """
        Callback executado quando o estimador publica novos dados.
        Apenas atualiza as variáveis internas com proteção de thread.
        """
        with self.data_lock:
            self.orientation = [data.qx, data.qy, data.qz, data.qw]
            self.position = [data.x, data.y, data.z]
            
    def _quaternion_to_opengl_matrix(self, q):
        """
        Converte um quaternião [x, y, z, w] em uma Matriz de Rotação 4x4
        compatível com OpenGL (Column-Major).
        
        Matemática: Esta matriz aplica a rotação definida pelo quaternião
        a qualquer vértice que for multiplicado por ela.
        """
        qx, qy, qz, qw = q
        
        # Pré-cálculos quadráticos para otimização
        qx2 = qx * qx; qy2 = qy * qy; qz2 = qz * qz
        qxy = qx * qy; qxz = qx * qz; qyz = qy * qz
        qwx = qw * qx; qwy = qw * qy; qwz = qw * qz
        
        # Matriz de Rotação Homogênea (baseada na Fórmula de Rodrigues para Quaterniões)
        matrix = [
            1.0 - 2.0 * (qy2 + qz2), 2.0 * (qxy + qwz), 2.0 * (qxz - qwy), 0.0,
            2.0 * (qxy - qwz), 1.0 - 2.0 * (qx2 + qz2), 2.0 * (qyz + qwx), 0.0,
            2.0 * (qxz + qwy), 2.0 * (qyz - qwx), 1.0 - 2.0 * (qx2 + qy2), 0.0,
            0.0, 0.0, 0.0, 1.0
        ]
        return matrix

    def _init_gl(self):
        """Configuração inicial da máquina de estado do OpenGL."""
        # Cria janela com buffer duplo (evita flickering)
        pygame.display.set_mode((self.width, self.height), DOUBLEBUF | OPENGL)
        pygame.display.set_caption("IMU Tracking: Orientation + Position")
        
        # Habilita teste de profundidade (para desenhar faces na ordem certa)
        glEnable(GL_DEPTH_TEST)
        glClearColor(0.05, 0.05, 0.05, 1.0) # Fundo cinza escuro
        
        # Configura a Lente da Câmera (Projeção)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        # FOV de 45 graus, Aspect Ratio da janela, Near clip 0.1, Far clip 100
        gluPerspective(45, (self.width / self.height), 0.1, 100.0)
        
        # Configura a Posição da Câmera (ModelView)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        # Câmera em (0, 5, 10), olhando para (0,0,0), com o eixo Y para cima
        gluLookAt(0, 5, 10, 0, 0, 0, 0, 1, 0)

    def _draw_grid(self):
        """Desenha um grid no chão para referência de movimento."""
        glLineWidth(1.0)
        glColor3f(0.3, 0.3, 0.3)  # Cinza
        
        grid_size = 20
        step = 2
        
        glBegin(GL_LINES)
        for i in range(-grid_size, grid_size + 1, step):
            # Linhas paralelas ao eixo X
            glVertex3f(-grid_size, 0, i)
            glVertex3f(grid_size, 0, i)
            # Linhas paralelas ao eixo Z
            glVertex3f(i, 0, -grid_size)
            glVertex3f(i, 0, grid_size)
        glEnd()

    def _draw_axes(self):
        """Desenha os eixos locais do cubo (RGB = XYZ)."""
        glLineWidth(3.0)
        glBegin(GL_LINES)
        # Eixo X (Vermelho)
        glColor3f(1.0, 0.0, 0.0); glVertex3f(0.0, 0.0, 0.0); glVertex3f(1.5, 0.0, 0.0)
        # Eixo Y (Verde)
        glColor3f(0.0, 1.0, 0.0); glVertex3f(0.0, 0.0, 0.0); glVertex3f(0.0, 1.5, 0.0)
        # Eixo Z (Azul)
        glColor3f(0.0, 0.0, 1.0); glVertex3f(0.0, 0.0, 0.0); glVertex3f(0.0, 0.0, 1.5)
        glEnd()

    def _draw_cube(self):
        """Desenha a geometria do cubo (Faces e Bordas)."""
        # 1. Desenha as faces sólidas
        glBegin(GL_QUADS)
        for i, surface in enumerate(self.surfaces):
            glColor3fv(self.colors[i])
            for vertex_index in surface:
                glVertex3fv(self.vertices[vertex_index])
        glEnd()
        
        # 2. Desenha as arestas (contorno branco)
        glColor3f(1.0, 1.0, 1.0)
        glLineWidth(1.0)
        glBegin(GL_LINES)
        for edge in self.edges:
            for vertex_index in edge:
                glVertex3fv(self.vertices[vertex_index])
        glEnd()

    def run(self):
        """Loop principal de renderização e captura de input."""
        pygame.init()
        self._init_gl()
        
        # Parâmetros de simulação (para quando usar o teclado)
        ACCEL_STEP = 0.5
        GYRO_STEP = 1.5
        
        running = True
        while running:
            # --- Tratamento de Eventos da Janela ---
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
            
            # --- Captura de Teclado (Simulação) ---
            # Permite controlar a física simulada se o modo "KEYBOARD" estiver ativo
            keys = pygame.key.get_pressed()
            
            sim_accel = [0.0, 0.0, 0.0]
            sim_gyro = [0.0, 0.0, 0.0]
            
            # WASD + QE (Aceleração Linear)
            if keys[pygame.K_w]: sim_accel[0] = ACCEL_STEP  # Frente
            if keys[pygame.K_s]: sim_accel[0] = -ACCEL_STEP # Trás
            if keys[pygame.K_a]: sim_accel[1] = ACCEL_STEP  # Esquerda
            if keys[pygame.K_d]: sim_accel[1] = -ACCEL_STEP # Direita
            if keys[pygame.K_q]: sim_accel[2] = ACCEL_STEP  # Cima
            if keys[pygame.K_e]: sim_accel[2] = -ACCEL_STEP # Baixo
            
            # IJKL + UO (Velocidade Angular)
            if keys[pygame.K_i]: sim_gyro[0] = GYRO_STEP    # Pitch +
            if keys[pygame.K_k]: sim_gyro[0] = -GYRO_STEP   # Pitch -
            if keys[pygame.K_l]: sim_gyro[1] = GYRO_STEP    # Roll +
            if keys[pygame.K_j]: sim_gyro[1] = -GYRO_STEP   # Roll -
            if keys[pygame.K_o]: sim_gyro[2] = GYRO_STEP    # Yaw +
            if keys[pygame.K_u]: sim_gyro[2] = -GYRO_STEP   # Yaw -
            
            # Publica inputs para o estimador processar (apenas útil no modo simulação)
            self.mediator.publish("sim_control", {
                'accel': sim_accel, 
                'gyro': sim_gyro
            })

            # --- Renderização ---
            with self.data_lock:
                # Converte quaternião atual para matriz OpenGL
                rotation_matrix = self._quaternion_to_opengl_matrix(self.orientation)
                x, y, z = self.position 
            
            # Limpa a tela e o buffer de profundidade
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
            
            # Desenha elementos estáticos
            self._draw_grid()

            # --- Transformações do Cubo ---
            glPushMatrix() # Salva a matriz atual (câmera)
            
            scale_pos = 5.0 # Amplifica o movimento para ficar visível
            
            # Mapeamento de Coordenadas
            # O sistema físico usa: X=Norte, Y=Leste, Z=Baixo (ou Cima)
            # O OpenGL usa: Y=Cima, X=Direita, Z=Fundo
            # Aqui fazemos a conversão visual:
            # OpenGL X = Físico X
            # OpenGL Y = Físico Z (Altura)
            # OpenGL Z = -Físico Y (Profundidade)
            glTranslatef(x * scale_pos, z * scale_pos, -y * scale_pos)
            
            # Rotaciona o cubo 90 graus para alinhar a face inicial com a câmera
            glRotatef(-90, 1, 0, 0)
            
            # Aplica a rotação calculada pelo EKF/Madgwick
            glMultMatrixf(rotation_matrix)
            
            # Desenha o objeto
            self._draw_cube()
            self._draw_axes()
            
            glPopMatrix() # Restaura a matriz (para não afetar o próximo frame)
            
            pygame.display.flip() # Atualiza a janela
            pygame.time.wait(10)  # Limita a ~100 FPS
            
        pygame.quit()
        print("Visualizador encerrado.")