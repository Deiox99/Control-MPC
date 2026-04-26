import numpy as np
import cvxpy as cp
import matplotlib.pyplot as plt

# Parámetros del robot
L = 0.25   # distancia entre ruedas (m)
r = 0.03   # radio de rueda (m)
a = 0.5    # amplitud del recorrido
omega = 0.2  # frecuencia de la trayectoria
N = 20     # horizonte de predicción
dt = 0.1   # paso de simulación
T = 40.0   # tiempo total de simulación

# Trayectoria deseada
def trayectoria(t):
    x_d = 2*a * np.sin(omega*t)
    y_d = 2*a * np.sin(2*omega*t)
    dx_d = 2*a*omega*np.cos(omega*t)
    dy_d = 4*a*omega*np.cos(2*omega*t)
    v_d = np.sqrt(dx_d**2 + dy_d**2)
    theta_d = np.arctan2(dy_d, dx_d)   # radianes
    return x_d, y_d, v_d, theta_d

# Control MPC simplificado (linealizado)
def control_mpc(x, y, theta, t):
    vr = cp.Variable(N)
    vl = cp.Variable(N)

    x_k, y_k, theta_k = x, y, theta
    cost = 0
    constraints = []

    cos_t = np.cos(theta_k)
    sin_t = np.sin(theta_k)

    for k in range(N):
        v = r/2 * (vr[k] + vl[k])
        omega_c = r/L * (vr[k] - vl[k])

        # Aproximación lineal
        x_k = x_k + v * cos_t * dt
        y_k = y_k + v * sin_t * dt
        theta_k = theta_k + omega_c * dt

        # Estado deseado
        x_d, y_d, v_d, theta_d = trayectoria(t + k*dt)

        # Costo
        cost += cp.square(x_k - x_d) + cp.square(y_k - y_d) + cp.square(theta_k - theta_d)

        # Restricciones
        constraints += [vr[k] <= 50, vr[k] >= -50]
        constraints += [vl[k] <= 50, vl[k] >= -50]

    prob = cp.Problem(cp.Minimize(cost), constraints)
    prob.solve()

    vr_opt = vr.value[0]
    vl_opt = vl.value[0]

    _, _, _, theta_d = trayectoria(t)
    theta_d_deg = np.degrees(theta_d)

    return {"vr": float(vr_opt), "vl": float(vl_opt), "theta_d_deg": float(theta_d_deg)}

# Simulación
x, y, theta = 0.0, 0.0, 0.0
traj_x, traj_y = [], []
traj_xd, traj_yd = [], []

t = 0.0
while t < T:
    control = control_mpc(x, y, theta, t)
    vr, vl, theta_d_deg = control["vr"], control["vl"], control["theta_d_deg"]

    # Dinámica real del robot
    v = (r/2) * (vr + vl)
    omega_c = (r/L) * (vr - vl)

    x = x + v * np.cos(theta) * dt
    y = y + v * np.sin(theta) * dt
    theta = theta + omega_c * dt

    traj_x.append(x)
    traj_y.append(y)

    # Trayectoria deseada en este instante
    x_d = 2*a * np.sin(omega*t)
    y_d = 2*a * np.sin(2*omega*t)
    traj_xd.append(x_d)
    traj_yd.append(y_d)

    t += dt

# Graficar comparación
plt.plot(traj_xd, traj_yd, 'r--', label="Trayectoria deseada (8 invertido)")
plt.plot(traj_x, traj_y, 'b', label="Trayectoria simulada (MPC)")
plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.title("Comparación trayectoria deseada vs simulada")
plt.legend()
plt.axis("equal")
plt.show()
