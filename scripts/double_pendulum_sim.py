import numpy as np
import matplotlib
matplotlib.use("GTKAgg")  
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, FFMpegWriter


# Clase que representa el sistema doble péndulo con un actuador en el segundo ángulo
class DoublePendulumSimulator:
    def __init__(self, L1=0.2, L2=0.25, M1=0.5, M2=2.0, g=9.81, dt=0.01, t_max=5.0,
                 b1=0.1, b2=0.4):
        self.L1, self.L2, self.M1, self.M2 = L1, L2, M1, M2
        self.g, self.dt, self.t_max = g, dt, t_max
        self.b1, self.b2 = b1, b2
        self.steps = int(t_max / dt)

    def dynamics(self, state, tau=0.0, noise_std=0.01):
        theta1, omega1, theta2, omega2 = state
        m1, m2, l1, l2, g = self.M1, self.M2, self.L1, self.L2, self.g
        b1, b2 = self.b1, self.b2

        # Restricciones físicas:
        #if np.abs(theta1) > np.pi / 2:
        #    return np.zeros(4)

        delta = theta2
        den1 = (m1 + m2) * l1**2 + m2 * l2**2 + 2 * m2 * l1 * l2 * np.cos(delta)
        den2 = m2 * l2**2 + m2 * l1 * l2 * np.cos(delta)

        domega1 = (-g * ((m1 + m2) * l1 * np.sin(theta1) + m2 * l2 * np.sin(theta1 + theta2)) +
                   m2 * l1 * l2 * omega2**2 * np.sin(delta) - b1 * omega1) / den1

        domega2 = (tau + m2 * l1 * l2 * omega1**2 * np.sin(delta) -
                   m2 * g * l2 * np.sin(theta1 + theta2) - b2 * omega2) / den2

        domega1 += np.random.normal(0, noise_std)
        domega2 += np.random.normal(0, noise_std)

        return np.array([omega1, domega1, omega2, domega2])

    def simulate(self, initial_state, controller=None):
        state = np.array(initial_state)
        history = [state.copy()]
        for _ in range(self.steps):
            tau = controller.compute_tau(state) if controller else 0.0
            k1 = self.dynamics(state, tau)
            k2 = self.dynamics(state + 0.5 * self.dt * k1, tau)
            k3 = self.dynamics(state + 0.5 * self.dt * k2, tau)
            k4 = self.dynamics(state + self.dt * k3, tau)
            state += (self.dt / 6) * (k1 + 2*k2 + 2*k3 + k4)
            history.append(state.copy())
        return np.array(history)

    def animate(self, history):
        fig, ax = plt.subplots()
        ax.set_xlim(-1, 1)
        ax.set_ylim(-0.6, 0.6)
        ax.set_aspect('equal')
        line1, = ax.plot([], [], 'or-', lw=4)
        line2, = ax.plot([], [], 'og-', lw=4)

        def update(i):
            theta1, _, theta2, _ = history[i]
            x0, y0 = 0, 0
            x1 = self.L1 * np.sin(theta1)
            y1 = -self.L1 * np.cos(theta1)
            x2 = x1 + self.L2 * np.sin(theta1 + theta2)
            y2 = y1 - self.L2 * np.cos(theta1 + theta2)
            line1.set_data([x0, x1], [y0, y1])
            line2.set_data([x1, x2], [y1, y2])
            return line1, line2

        ani = FuncAnimation(fig, update, frames=len(history), interval=1000*self.dt, blit=True)
        ani.show()
        #ani.save("simulacion_controlada.mp4", writer=FFMpegWriter(fps=30))



# ---------------------
# CONTROLADOR SWING-UP
# ---------------------
class SwingUpController:
    def __init__(self, simulator):
        self.sim = simulator
        self.target_energy = self.compute_target_energy()

    def compute_target_energy(self):
        # Energía total deseada (cuando está arriba)
        m1, m2, g = self.sim.M1, self.sim.M2, self.sim.g
        l1, l2 = self.sim.L1, self.sim.L2
        PE1 = (m1 + m2) * g * l1
        PE2 = m2 * g * l2
        return PE1 + PE2

    def total_energy(self, state):
        theta1, omega1, theta2, omega2 = state
        m1, m2, l1, l2, g = self.sim.M1, self.sim.M2, self.sim.L1, self.sim.L2, self.sim.g

        y1 = -l1 * np.cos(theta1)
        y2 = y1 - l2 * np.cos(theta1 + theta2)

        v1_sq = (l1 * omega1)**2
        v2_sq = v1_sq + (l2 * omega2)**2 + 2 * l1 * l2 * omega1 * omega2 * np.cos(theta2)

        KE = 0.5 * (m1 + m2) * v1_sq + 0.5 * m2 * v2_sq
        PE = (m1 + m2) * g * (y1 + l1) + m2 * g * (y2 + l2)

        return KE + PE

    def compute_control(self, state):
        theta1, omega1, theta2, omega2 = state

        # Seguridad: evitar que atraviese el suelo
        if np.abs(theta1) > np.pi / 2:
            return 0.0

        E = self.total_energy(state)
        dE = self.target_energy - E

        k = 10.0  # ganancia de energía
        torque = k * dE * np.sign(omega2)

        return torque


# --------------------------
# MANEJADOR DE SIMULACIÓN
# --------------------------
class SimulationManager:
    def __init__(self):
        self.simulator = DoublePendulumSimulator()
        self.controller = SwingUpController(self.simulator)

    def run(self, initial_state):
        state = np.array(initial_state)
        history = [state.copy()]
        for _ in range(self.simulator.steps):
            tau = self.controller.compute_control(state)
            k1 = self.simulator.dynamics(state, tau)
            k2 = self.simulator.dynamics(state + 0.5 * self.simulator.dt * k1, tau)
            k3 = self.simulator.dynamics(state + 0.5 * self.simulator.dt * k2, tau)
            k4 = self.simulator.dynamics(state + self.simulator.dt * k3, tau)
            state += (self.simulator.dt / 6) * (k1 + 2*k2 + 2*k3 + k4)
            history.append(state.copy())
        return np.array(history)

    def animate(self, history):
        fig, ax = plt.subplots()
        ax.set_xlim(-1, 1)
        ax.set_ylim(-0.6, 0.6)
        ax.set_aspect('equal')
        line1, = ax.plot([], [], 'or-', lw=4)
        line2, = ax.plot([], [], 'og-', lw=4)

        def update(i):
            theta1, _, theta2, _ = history[i]
            x0, y0 = 0, 0
            x1 = self.simulator.L1 * np.sin(theta1)
            y1 = -self.simulator.L1 * np.cos(theta1)
            x2 = x1 + self.simulator.L2 * np.sin(theta1 + theta2)
            y2 = y1 - self.simulator.L2 * np.cos(theta1 + theta2)
            line1.set_data([x0, x1], [y0, y1])
            line2.set_data([x1, x2], [y1, y2])
            return line1, line2

        ani = FuncAnimation(fig, update, frames=len(history), interval=1000*self.simulator.dt, blit=True)
        ani.save("simulacion_swingup.mp4", writer=FFMpegWriter(fps=30))



def main():
    # Crear el simulador y ejecutar con condiciones iniciales
    sim_mgr = SimulationManager()
    initial_state = [0.0, 0.0, 0.0, 0.0]

    trajectory = sim_mgr.run(initial_state)

    sim_mgr.animate(trajectory)

if __name__ == "__main__":
    main()