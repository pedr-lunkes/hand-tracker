"""
ekf_plotter.py

Visualizador estatístico para o Filtro de Kalman Estendido.
Plota a Distribuição de Probabilidade (PDF) dos 4 componentes do quaternião.
Permite visualizar se o filtro está "confiante" (curva fina/alta) ou "incerto" (curva larga/baixa).
"""

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import threading
from mediator import Mediator 

class GaussianPlotter:
    """
    Assina o tópico "orientation" e desenha curvas normais baseadas
    na média e variância fornecidas pelo EKF.
    """

    def __init__(self, mediator: Mediator):
        self.mediator = mediator
        self.mediator.subscribe("orientation", self.update_data)
        
        self.latest_data = None
        self.lock = threading.Lock()
        
        # Configuração da Janela de Plotagem (2x2 gráficos)
        self.fig, self.axs = plt.subplots(2, 2, figsize=(12, 8))
        self.fig.suptitle('EKF: Distribuição de Incerteza (Quaterniões)', fontsize=16)
        
        # Eixo X fixo de -1.2 a 1.2 (já que quaterniões normalizados vão de -1 a 1)
        self.x_range = np.linspace(-1.2, 1.2, 300)
        
        titles = ['Distribuição q_w (Escalar)', 'Distribuição q_x', 'Distribuição q_y', 'Distribuição q_z']
        self.lines = []
        
        # Inicializa as linhas vazias nos 4 subplots
        for i, ax in enumerate(self.axs.flat):
            line, = ax.plot(self.x_range, np.zeros_like(self.x_range), lw=2, color='blue')
            self.lines.append(line)
            ax.set_title(titles[i])
            ax.set_xlim(-1.2, 1.2)
            ax.grid(True, linestyle='--', alpha=0.6)
        
        self.axs[1, 0].set_xlabel('Valor do Estado')
        self.axs[1, 1].set_xlabel('Valor do Estado')
        self.axs[0, 0].set_ylabel('Densidade de Probabilidade')
        self.axs[1, 0].set_ylabel('Densidade de Probabilidade')

    def update_data(self, data):
        """Callback: Recebe o objeto OrientationData contendo médias e variâncias."""
        with self.lock:
            self.latest_data = data
            
    def _normal_dist(self, x, mu, sigma):
        """
        Calcula a Função de Densidade de Probabilidade (PDF) Gaussiana.
        f(x) = (1 / (sigma * sqrt(2pi))) * exp(...)
        
        Args:
            x: Pontos do eixo X.
            mu: Média (o valor estimado do estado).
            sigma: Desvio Padrão (raiz quadrada da variância).
        """
        # Proteção contra divisão por zero.
        if sigma < 1e-6:
            sigma = 1e-6
            
        return (1.0 / (sigma * np.sqrt(2 * np.pi))) * \
               np.exp(-0.5 * ((x - mu) / sigma) ** 2)

    def _animate(self, i):
        """Função chamada repetidamente pelo Matplotlib para atualizar o gráfico."""
        with self.lock:
            local_data = self.latest_data
            
        # Se ainda não recebeu dados, não faz nada
        if local_data is None:
            return self.lines
            
        # Extrai médias (Estados)
        means = [local_data.qw, local_data.qx, local_data.qy, local_data.qz]
        
        # Extrai variâncias (Diagonal da Matriz P)
        # Nota: O objeto data deve ter esses campos (o EKF popula, o Madgwick manda 0)
        variances = [local_data.var_qw, local_data.var_qx, 
                     local_data.var_qy, local_data.var_qz]
        
        max_y = 0 
        
        # Atualiza as 4 curvas
        for k in range(4):
            mu = means[k]
            # Desvio Padrão = Raiz da Variância
            sigma = np.sqrt(variances[k])
            
            # Recalcula a curva de sino
            y_data = self._normal_dist(self.x_range, mu, sigma)
            self.lines[k].set_ydata(y_data)
            
            # Descobre o pico para ajustar a escala Y automaticamente
            peak_y = self._normal_dist(mu, mu, sigma)
            if peak_y > max_y:
                max_y = peak_y
        
        # Ajusta o limite vertical para o gráfico não "cortar" o topo da curva
        # Limita a 50.0 para evitar escalas infinitas se a variância for 0
        display_max = min(max_y * 1.15, 50.0)
        for ax in self.axs.flat:
            ax.set_ylim(0, display_max + 0.1) 
            
        return self.lines

    def run(self):
        """
        Inicia o loop de animação do Matplotlib.
        """
        ani = animation.FuncAnimation(self.fig, self._animate, 
                                      interval=50, # Atualiza a cada 50ms (20 FPS)
                                      blit=False)  # Blit False é mais compatível
        
        plt.tight_layout(rect=[0, 0.03, 1, 0.95])
        plt.show()