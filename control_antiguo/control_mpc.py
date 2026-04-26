"""
control_mpc.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
MPC linealizado para robot diferencial con trayectoria Lissajous (figura ∞).

El modelo cinemático se linealiza en torno al heading actual θ_k (ángulo fijo
durante el horizonte), lo que permite formular el problema como un QP convexo
soluble eficientemente con CVXPY.

Flujo de cada ciclo:
  1. Trayectoria → obtiene referencia deseada (x_d, y_d, θ_d) en cada paso k
  2. Control MPC → resuelve QP con N pasos, retorna (v_r, v_l) óptimos
  3. Dinámica → integra el modelo cinemático con la entrada óptima
"""
import numpy as np
import cvxpy as cp
import matplotlib.pyplot as plt

# ══════════════════════════════════════════════════
#  PARÁMETROS DEL ROBOT
# ══════════════════════════════════════════════════
L:  float = 0.25   # [m]   Distancia entre ruedas (track width).
r:  float = 0.03   # [m]   Radio de cada rueda motriz.

# ══════════════════════════════════════════════════
#  PARÁMETROS DE LA TRAYECTORIA (Lissajous ∞)
# ══════════════════════════════════════════════════
a:     float = 0.5    # [m]     Amplitud de la figura. Controla el tamaño del ∞.
omega: float = 0.2    # [rad/s] Frecuencia angular de la trayectoria.
                       #          Período: T_curva = 2π/ω ≈ 31.4 s.

# ══════════════════════════════════════════════════
#  PARÁMETROS DEL CONTROLADOR MPC
# ══════════════════════════════════════════════════
N:  int   = 20    # [pasos] Horizonte de predicción.
                   #          Más alto = mejor anticipación, más tiempo de cómputo.
dt: float = 0.1   # [s]     Período de muestreo del MPC y de la simulación.

# ══════════════════════════════════════════════════
#  PARÁMETROS DE LA SIMULACIÓN
# ══════════════════════════════════════════════════
T: float = 40.0   # [s]  Tiempo total de simulación.


# ── Generador de trayectoria de referencia ─────────────────────

def trayectoria(t: float) -> tuple:
    """
    Calcula el estado de referencia deseado en el instante t.

    La curva es una Lissajous con relación de frecuencias 1:2,
    que genera una figura ∞ (similar a la Lemniscata de Gerono).

    Entrada:  t [s] — instante de evaluación
    Salida:   (x_d, y_d, v_d, theta_d)
                x_d     [m]     posición X deseada
                y_d     [m]     posición Y deseada
                v_d     [m/s]   velocidad lineal requerida (módulo del tangente)
                theta_d [rad]   heading deseado (dirección tangencial)
    """
    x_d:  float = 2.0 * a * np.sin(omega * t)          # [m]   x de la curva
    y_d:  float = 2.0 * a * np.sin(2.0 * omega * t)    # [m]   y de la curva

    # Derivadas temporales de la posición → vector velocidad tangente
    dx_d: float = 2.0 * a * omega * np.cos(omega * t)          # [m/s]
    dy_d: float = 4.0 * a * omega * np.cos(2.0 * omega * t)    # [m/s]

    v_d:     float = np.hypot(dx_d, dy_d)   # [m/s]  módulo de la velocidad tangente
    theta_d: float = np.arctan2(dy_d, dx_d) # [rad]  heading tangencial en (-π, π]

    return x_d, y_d, v_d, theta_d


# ── Controlador MPC (QP linealizado) ──────────────────────────

def control_mpc(
    x:     float,   # [m]   Posición X actual del robot
    y:     float,   # [m]   Posición Y actual del robot
    theta: float,   # [rad] Orientación actual del robot
    t:     float,   # [s]   Tiempo actual de simulación
) -> dict:
    """
    Resuelve el MPC linealizado y retorna la acción de control óptima.

    El modelo se linealiza en torno al heading actual θ (constante durante
    el horizonte), transformando el QP no-lineal en uno convexo resoluble.

    Variables de decisión:
        vr[k], vl[k]  — velocidades angulares de rueda derecha/izquierda [rad/s]
                          k = 0, 1, ..., N-1

    Entrada:
        x, y, theta — estado actual del robot
        t           — tiempo actual para evaluar la trayectoria de referencia

    Salida: dict con claves:
        'vr'        [rad/s]  velocidad angular rueda derecha óptima (paso k=0)
        'vl'        [rad/s]  velocidad angular rueda izquierda óptima (paso k=0)
        'theta_d_deg' [°]   heading deseado en el instante actual (para log)
    """
    # Variables de decisión del QP: velocidades de rueda en cada paso del horizonte
    vr: cp.Variable = cp.Variable(N)   # [rad/s] velocidad angular rueda derecha, k=0..N-1
    vl: cp.Variable = cp.Variable(N)   # [rad/s] velocidad angular rueda izquierda, k=0..N-1

    # Estado inicial del horizonte de propagación
    x_k:     float = x
    y_k:     float = y
    theta_k: float = theta

    # Linearización: cos/sin se calculan una sola vez en θ_k actual
    cos_t: float = np.cos(theta_k)   # componente X del vector de avance
    sin_t: float = np.sin(theta_k)   # componente Y del vector de avance

    cost:        cp.Expression = 0   # acumulador de costo (suma sobre el horizonte)
    constraints: list          = []  # restricciones de desigualdad (límites de velocidad)

    for k in range(N):
        # Conversión velocidades de rueda → velocidades del robot
        v:       cp.Expression = (r / 2.0) * (vr[k] + vl[k])    # [m/s]   lineal
        omega_c: cp.Expression = (r / L)   * (vr[k] - vl[k])    # [rad/s] angular

        # Propagación linealizada del modelo (Euler con θ fijo)
        x_k     = x_k + v * cos_t * dt      # [m]   X predicho en paso k+1
        y_k     = y_k + v * sin_t * dt      # [m]   Y predicho en paso k+1
        theta_k = theta_k + omega_c * dt    # [rad] θ predicho en paso k+1

        # Estado de referencia en el paso k+1
        x_d, y_d, v_d, theta_d = trayectoria(t + k * dt)

        # Función de costo cuadrática: error de posición + error de heading
        cost += cp.square(x_k - x_d) + cp.square(y_k - y_d) + cp.square(theta_k - theta_d)

        # Restricciones de velocidad de rueda [rad/s]
        constraints += [vr[k] <= 50, vr[k] >= -50]   # límite superior/inferior rueda derecha
        constraints += [vl[k] <= 50, vl[k] >= -50]   # límite superior/inferior rueda izquierda

    # Resuelve el QP convexo
    prob = cp.Problem(cp.Minimize(cost), constraints)
    prob.solve()

    # Extrae primera acción de control (receding horizon: solo k=0 se aplica)
    vr_opt: float = float(vr.value[0])   # [rad/s] velocidad rueda derecha a aplicar
    vl_opt: float = float(vl.value[0])   # [rad/s] velocidad rueda izquierda a aplicar

    # Heading deseado actual (solo para logging)
    _, _, _, theta_d_now = trayectoria(t)
    theta_d_deg: float = np.degrees(theta_d_now)   # [°] para visualización en logs

    return {"vr": vr_opt, "vl": vl_opt, "theta_d_deg": theta_d_deg}


# ══════════════════════════════════════════════════
#  BUCLE DE SIMULACIÓN
# ══════════════════════════════════════════════════

# Estado inicial del robot
x:     float = 0.0   # [m]   Posición inicial X
y:     float = 0.0   # [m]   Posición inicial Y
theta: float = 0.0   # [rad] Orientación inicial

# Buffers de historial para el plot
traj_x:  list = []   # [m]  Posiciones X del robot a lo largo del tiempo
traj_y:  list = []   # [m]  Posiciones Y del robot a lo largo del tiempo
traj_xd: list = []   # [m]  Posiciones X de la trayectoria deseada
traj_yd: list = []   # [m]  Posiciones Y de la trayectoria deseada

t: float = 0.0   # [s] Tiempo de simulación acumulado

while t < T:
    # Resuelve MPC y obtiene velocidades de rueda óptimas
    control = control_mpc(x, y, theta, t)
    vr_cmd: float = control["vr"]   # [rad/s] comando rueda derecha
    vl_cmd: float = control["vl"]   # [rad/s] comando rueda izquierda

    # Convierte velocidades de rueda a velocidades del robot
    v_robot:     float = (r / 2.0) * (vr_cmd + vl_cmd)   # [m/s]   velocidad lineal
    omega_robot: float = (r / L)   * (vr_cmd - vl_cmd)   # [rad/s] velocidad angular

    # Integración Euler del modelo cinemático (dinámica real del robot)
    x     = x     + v_robot * np.cos(theta) * dt   # [m]   nueva posición X
    y     = y     + v_robot * np.sin(theta) * dt   # [m]   nueva posición Y
    theta = theta + omega_robot * dt                # [rad] nueva orientación

    # Almacena posición real del robot
    traj_x.append(x)
    traj_y.append(y)

    # Almacena posición de referencia correspondiente a este instante
    x_d: float = 2.0 * a * np.sin(omega * t)        # [m] x deseada
    y_d: float = 2.0 * a * np.sin(2.0 * omega * t)  # [m] y deseada
    traj_xd.append(x_d)
    traj_yd.append(y_d)

    t += dt   # avanza el tiempo de simulación

# ══════════════════════════════════════════════════
#  VISUALIZACIÓN
# ══════════════════════════════════════════════════
fig, ax = plt.subplots(figsize=(8, 6))
ax.plot(traj_xd, traj_yd, 'r--', lw=1.5, label="Trayectoria deseada (∞)")
ax.plot(traj_x,  traj_y,  'b',   lw=1.5, label="Trayectoria simulada (MPC)")
ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")
ax.set_title("Comparación trayectoria deseada vs simulada (MPC linealizado)")
ax.set_aspect('equal')
ax.legend()
ax.grid(True, alpha=0.3)
plt.tight_layout()
plt.show()
