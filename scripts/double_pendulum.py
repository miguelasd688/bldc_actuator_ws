import sympy as sp


def main():
    # Variables de tiempo y funciones
    t = sp.symbols('t')
    theta1 = sp.Function('theta1')(t)
    theta2 = sp.Function('theta2')(t)
    dtheta1 = theta1.diff(t)
    dtheta2 = theta2.diff(t)
    ddtheta1 = theta1.diff(t, 2)
    ddtheta2 = theta2.diff(t, 2)

    # Parámetros físicos
    L1, L2, M1, M2, g, tau = sp.symbols('L1 L2 M1 M2 g tau')

    # Posición del centro de masa de cada barra
    x1 = (L1/2) * sp.sin(theta1)
    y1 = -(L1/2) * sp.cos(theta1)
    x2 = L1 * sp.sin(theta1) + (L2/2) * sp.sin(theta1 + theta2)
    y2 = -L1 * sp.cos(theta1) - (L2/2) * sp.cos(theta1 + theta2)

    # Velocidades (por derivación total)
    vx1 = x1.diff(t)
    vy1 = y1.diff(t)
    vx2 = x2.diff(t)
    vy2 = y2.diff(t)

    v1_sq = vx1**2 + vy1**2
    v2_sq = vx2**2 + vy2**2

    # Momentos de inercia
    I1 = (1/12) * M1 * L1**2
    I2 = (1/12) * M2 * L2**2

    # Energía cinética
    T1 = (1/2) * M1 * v1_sq + (1/2) * I1 * dtheta1**2
    T2 = (1/2) * M2 * v2_sq + (1/2) * I2 * (dtheta1 + dtheta2)**2
    T = sp.simplify(T1 + T2)

    # Energía potencial
    V1 = M1 * g * y1
    V2 = M2 * g * y2
    V = V1 + V2

    # Lagrangiano
    L = T - V

    # Derivadas parciales para ecuaciones de Lagrange
    dL_dtheta1 = L.diff(theta1)
    dL_ddtheta1 = L.diff(dtheta1)
    d_dt_dL_ddtheta1 = dL_ddtheta1.diff(t)

    dL_dtheta2 = L.diff(theta2)
    dL_ddtheta2 = L.diff(dtheta2)
    d_dt_dL_ddtheta2 = dL_ddtheta2.diff(t)

    # Ecuaciones de Lagrange
    eq1 = sp.Eq(d_dt_dL_ddtheta1 - dL_dtheta1, 0)
    eq2 = sp.Eq(d_dt_dL_ddtheta2 - dL_dtheta2, tau)

    print(eq1)
    print(eq2)  # Mostramos las ecuaciones no lineales completas (planta)




if __name__ == "__main__":
    main()