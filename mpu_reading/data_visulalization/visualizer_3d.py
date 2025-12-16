"""
visualizer_3d.py

A subscriber that visualizes orientation and position data as a 3D cube
using Pygame and PyOpenGL.
"""

import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import threading
from mediator import Mediator

class CubeVisualizer:
    """
    Subscribes to the Mediator for "orientation" data and displays the
    orientation and position as a moving 3D cube.
    """
    def __init__(self, mediator: Mediator):
        self.mediator = mediator
        self.mediator.subscribe("orientation", self.update_orientation)
        
        # Quaternion [qx, qy, qz, qw]
        self.orientation = [0.0, 0.0, 0.0, 1.0]
        # Position [x, y, z]
        self.position = [0.0, 0.0, 0.0]
        
        self.data_lock = threading.Lock()
        
        self.width = 800
        self.height = 600
        
        # Cube definitions
        self.vertices = (
            (1, -1, -1), (1, 1, -1), (-1, 1, -1), (-1, -1, -1),
            (1, -1, 1), (1, 1, 1), (-1, -1, 1), (-1, 1, 1)
        )
        self.edges = (
            (0,1), (0,3), (0,4), (2,1), (2,3), (2,7),
            (6,3), (6,4), (6,7), (5,1), (5,4), (5,7)
        )
        self.colors = (
            (1,0,0), (0,1,0), (0,0,1), (1,1,0), (0,1,1), (1,0,1)
        )
        self.surfaces = (
            (0,1,2,3), (3,2,7,6), (6,7,5,4),
            (4,5,1,0), (1,5,7,2), (4,0,3,6)
        )

    def update_orientation(self, data):
        """Callback function to receive new orientation and position data."""
        with self.data_lock:
            # Atualiza quaternião
            self.orientation = [data.qx, data.qy, data.qz, data.qw]
            # Atualiza posição (assumindo que data.x, data.y, data.z existem)
            self.position = [data.x, data.y, data.z]
            
    def _quaternion_to_opengl_matrix(self, q):
        """
        Converts a quaternion [x, y, z, w] to a 4x4
        OpenGL-compatible rotation matrix (column-major).
        """
        qx, qy, qz, qw = q
        
        qx2 = qx * qx; qy2 = qy * qy; qz2 = qz * qz
        qxy = qx * qy; qxz = qx * qz; qyz = qy * qz
        qwx = qw * qx; qwy = qw * qy; qwz = qw * qz
        
        matrix = [
            1.0 - 2.0 * (qy2 + qz2), 2.0 * (qxy + qwz), 2.0 * (qxz - qwy), 0.0,
            2.0 * (qxy - qwz), 1.0 - 2.0 * (qx2 + qz2), 2.0 * (qyz + qwx), 0.0,
            2.0 * (qxz + qwy), 2.0 * (qyz - qwx), 1.0 - 2.0 * (qx2 + qy2), 0.0,
            0.0, 0.0, 0.0, 1.0
        ]
        return matrix

    def _init_gl(self):
        """Initializes OpenGL settings."""
        pygame.display.set_mode((self.width, self.height), DOUBLEBUF | OPENGL)
        pygame.display.set_caption("IMU Tracking: Orientation + Position")
        
        glEnable(GL_DEPTH_TEST)
        glClearColor(0.05, 0.05, 0.05, 1.0) # Fundo levemente mais escuro
        
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        # Aumentei o Far Plane para 100.0 para permitir ver se afastar mais
        gluPerspective(45, (self.width / self.height), 0.1, 100.0)
        
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

        gluLookAt(0, 5, 10,  # Eye (X, Y, Z)
                  0, 0, 0,   # Center (looking at origin)
                  0, 1, 0)   # Up vector

    def _draw_grid(self):
        """Creates a static grid to instatiate the cube """
        glLineWidth(1.0)
        glColor3f(0.3, 0.3, 0.3)  # gray
        
        grid_size = 20
        step = 2
        
        glBegin(GL_LINES)
        for i in range(-grid_size, grid_size + 1, step):
            glVertex3f(-grid_size, 0, i)
            glVertex3f(grid_size, 0, i)
            glVertex3f(i, 0, -grid_size)
            glVertex3f(i, 0, grid_size)
        glEnd()

    def _draw_axes(self):
        """Draws local axes."""
        glLineWidth(3.0)
        glBegin(GL_LINES)
        # X (Red)
        glColor3f(1.0, 0.0, 0.0); glVertex3f(0.0, 0.0, 0.0); glVertex3f(1.5, 0.0, 0.0)
        # Y (Green)
        glColor3f(0.0, 1.0, 0.0); glVertex3f(0.0, 0.0, 0.0); glVertex3f(0.0, 1.5, 0.0)
        # Z (Blue)
        glColor3f(0.0, 0.0, 1.0); glVertex3f(0.0, 0.0, 0.0); glVertex3f(0.0, 0.0, 1.5)
        glEnd()

    def _draw_cube(self):
        """Draws the 3D cube."""
        glBegin(GL_QUADS)
        for i, surface in enumerate(self.surfaces):
            glColor3fv(self.colors[i])
            for vertex_index in surface:
                glVertex3fv(self.vertices[vertex_index])
        glEnd()
        
        glColor3f(1.0, 1.0, 1.0)
        glLineWidth(1.0)
        glBegin(GL_LINES)
        for edge in self.edges:
            for vertex_index in edge:
                glVertex3fv(self.vertices[vertex_index])
        glEnd()

    def run(self):
        """The main rendering loop + Input Capture."""
        pygame.init()
        self._init_gl()
        
        # ---- Keyboard simulation parameters ----
        ACCEL_STEP = 0.5
        GYRO_STEP = 1.5
        
        running = True
        while running:
            # --- Window events ---
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                if event.type == pygame.KEYDOWN:  # You can exit with Escape :D
                    if event.key == pygame.K_ESCAPE:
                        running = False
            
            # Captures the inputs and publishes them in the "sim_controll" topic
            keys = pygame.key.get_pressed()
            
            sim_accel = [0.0, 0.0, 0.0]
            sim_gyro = [0.0, 0.0, 0.0]
            
            # Acceleration controls (WASD QE)
            if keys[pygame.K_w]: sim_accel[0] = ACCEL_STEP  # +X
            if keys[pygame.K_s]: sim_accel[0] = -ACCEL_STEP # -X
            if keys[pygame.K_a]: sim_accel[1] = ACCEL_STEP  # +Y (Esq)
            if keys[pygame.K_d]: sim_accel[1] = -ACCEL_STEP # -Y (Dir)
            if keys[pygame.K_q]: sim_accel[2] = ACCEL_STEP  # +Z (Cima)
            if keys[pygame.K_e]: sim_accel[2] = -ACCEL_STEP # -Z (Baixo)
            
            # Giro controlls (IJKL UO)
            if keys[pygame.K_i]: sim_gyro[0] = GYRO_STEP    # +Pitch
            if keys[pygame.K_k]: sim_gyro[0] = -GYRO_STEP   # -Pitch
            if keys[pygame.K_l]: sim_gyro[1] = GYRO_STEP    # +Roll
            if keys[pygame.K_j]: sim_gyro[1] = -GYRO_STEP   # -Roll
            if keys[pygame.K_o]: sim_gyro[2] = GYRO_STEP    # +Yaw
            if keys[pygame.K_u]: sim_gyro[2] = -GYRO_STEP   # -Yaw
            
            # Sends the inputs to the mediator
            self.mediator.publish("sim_control", {
                'accel': sim_accel, 
                'gyro': sim_gyro
            })

            # ---- Rendering ----
            with self.data_lock:
                rotation_matrix = self._quaternion_to_opengl_matrix(self.orientation)
                x, y, z = self.position 
            
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
            self._draw_grid()

            glPushMatrix()
            scale = 5.0
            glTranslatef(x * scale, z * scale, -y * scale)  # Makes the z axis up, for better visualization
            glRotatef(-90, 1, 0, 0)
            
            glMultMatrixf(rotation_matrix)
            self._draw_cube()
            self._draw_axes()
            glPopMatrix()
            
            pygame.display.flip()
            pygame.time.wait(10) # ~100 FPS
            
        pygame.quit()
        print("Visualizer stopped.")