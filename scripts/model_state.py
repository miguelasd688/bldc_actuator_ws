import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp


class InvertedPendulumSimulator:
    def __init__(self, m=0.8, h=0.24, w=0.09, g=9.81):
        # Parámetros físicos del sistema
        self.m = m
        self.h = h
        self.w = w
        self.g = g

        # Momento de inercia aproximado del cuerpo (bloque rígido)
        self.I = (1 / 12) * m * (w**2 + h**2)
        self.l = h / 2  # Distancia del eje al centro de masa

    def torque_input(self, t):
        """Función de entrada de torque (puede ser modificada)"""
        return 0.01  # Nm

    def dynamics(self, t, state):
        """Ecuaciones diferenciales del péndulo invertido"""
        phi, dphi = state
        tau = self.torque_input(t)
        ddphi = (tau - self.m * self.g * self.l * np.sin(phi)) / self.I
        return [dphi, ddphi]

    def simulate(self, x0, t_span=(0, 5), num_points=1000):
        """Simula el sistema desde condiciones iniciales x0"""
        t_eval = np.linspace(*t_span, num_points)
        sol = solve_ivp(self.dynamics, t_span, x0, t_eval=t_eval)
        return sol.t, sol.y

    def plot_results(self, t, states):
        """Grafica los resultados de la simulación"""
        phi, dphi = states
        plt.figure(figsize=(10, 5))
        plt.plot(t, np.rad2deg(phi), label='Ángulo (grados)')
        plt.plot(t, np.rad2deg(dphi), label='Velocidad angular (°/s)')
        plt.xlabel('Tiempo (s)')
        plt.ylabel('Respuesta angular')
        plt.title('Simulación del Péndulo Invertido')
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.show()


def main():
    simulator = InvertedPendulumSimulator()
    x0 = [0.05, 0.0]  # Estado inicial: 0.05 rad de inclinación
    t, states = simulator.simulate(x0)
    simulator.plot_results(t, states)


if __name__ == "__main__":
    main()