"""
visualizer_3d.py

A subscriber that visualizes orientation data as a 3D cube
using Pygame and PyOpenGL.
"""

import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import numpy as np
import threading
import math
from orientation_mediator import Mediator # <-- CHANGED

class CubeVisualizer:
    """
    Subscribes to the Mediator for "orientation" data and displays the
    orientation as a rotating 3D cube.
    """
    
    def __init__(self, mediator: Mediator): # <-- CHANGED
        self.mediator = mediator
        self.mediator.subscribe("orientation", self.update_orientation) # <-- CHANGED
        
        # Quaternion [qx, qy, qz, qw]
        self.orientation = [0.0, 0.0, 0.0, 1.0]
        self.orientation_lock = threading.Lock()
        
        self.width = 800
        self.height = 600
        
        # ... (rest of the file is identical) ...
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
        """Callback function to receive new orientation data."""
        with self.orientation_lock:
            self.orientation = [data.qx, data.qy, data.qz, data.qw]
            
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
        pygame.display.set_caption("MPU6050 Orientation Visualizer")
        
        glEnable(GL_DEPTH_TEST)
        glClearColor(0.1, 0.1, 0.1, 1.0)
        
        glMatrixMode(GL_PROJECTION)
        gluPerspective(45, (self.width / self.height), 0.1, 50.0)
        
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        glTranslatef(0.0, 0.0, -7.0) 

    def _draw_axes(self):
        """Draws X (Red), Y (Green), Z (Blue) axes."""
        glLineWidth(2.0)
        glBegin(GL_LINES)
        glColor3f(1.0, 0.0, 0.0); glVertex3f(0.0, 0.0, 0.0); glVertex3f(2.0, 0.0, 0.0)
        glColor3f(0.0, 1.0, 0.0); glVertex3f(0.0, 0.0, 0.0); glVertex3f(0.0, 2.0, 0.0)
        glColor3f(0.0, 0.0, 1.0); glVertex3f(0.0, 0.0, 0.0); glVertex3f(0.0, 0.0, 2.0)
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
        glLineWidth(2.0)
        glBegin(GL_LINES)
        for edge in self.edges:
            for vertex_index in edge:
                glVertex3fv(self.vertices[vertex_index])
        glEnd()

    def run(self):
        """The main rendering loop."""
        pygame.init()
        self._init_gl()
        
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                        
            with self.orientation_lock:
                rotation_matrix = self._quaternion_to_opengl_matrix(self.orientation)
            
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
            glPushMatrix()
            glMultMatrixf(rotation_matrix)
            self._draw_cube()
            self._draw_axes()
            glPopMatrix()
            
            pygame.display.flip()
            pygame.time.wait(10)
            
        pygame.quit()
        print("Visualizer stopped.")