"""
simulation/sim_robot.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Simulador de robot diferencial en Python puro.
Permite probar y ajustar el MPC sin hardware físico.

Imita la interfaz de UARTHandler + StateEstimator para que
main.py funcione igual en modo simulación (--sim).

Uso:
    python main.py --sim
    python simulation/sim_robot.py   # prueba standalone con plot
"""
import numpy as np
import time
import logging

log = logging.getLogger(__name__)

# ── Magnitudes de ruido del proceso ────────────────────────────
# Representan imperfecciones físicas reales (deslizamiento, vibración).
# Aumentar para simular peores condiciones de tracción.
_NOISE_POS:   float = 0.0005   # [m]     Desviación estándar del ruido de posición
                                 #          por paso de integración.
_NOISE_THETA: float = 0.002    # [rad]   Desviación estándar del ruido de heading
                                 #          por paso de integración.
_NOISE_GYRO:  float = 0.005    # [rad/s] Ruido aditivo en la lectura simulada de gz
                                 #          (no usado en la integración principal).


class SimRobot:
    """
    Robot unicycle simulado con ruido gaussiano opcional.

    Interfaz compatible con StateEstimator.state para que
    el bucle de control no necesite cambios.
    """

    def __init__(
        self,
        x0:     float = 0.30,        # [m]   Posición inicial X. Por defecto: inicio
                                      #        de la lemniscata (A·cos(0) = A).
        y0:     float = 0.0,          # [m]   Posición inicial Y.
        theta0: float = np.pi / 2.0,  # [rad] Orientación inicial. π/2 = mirando en +Y,
                                      #        que es la dirección tangencial en t=0.
        dt:     float = 0.05,         # [s]   Período de integración. Debe coincidir con DT.
        noise:  bool  = True,         # Si True, añade ruido gaussiano al proceso.
                                      # Poner False para verificar el MPC sin perturbaciones.
    ):
        self.dt:    float = dt      # [s]   Período de integración (guardado para step)
        self.noise: bool  = noise   # Habilita/deshabilita ruido del proceso

        # ── Estado interno (ground truth, no observable desde fuera) ──
        self._x:     float = x0      # [m]   Posición X real del robot
        self._y:     float = y0      # [m]   Posición Y real del robot
        self._theta: float = theta0  # [rad] Orientación real del robot (-π, π]

        # ── Historial para visualización ──────────────────────
        self.history_x: list = [x0]   # [m]   Lista de posiciones X en cada paso
        self.history_y: list = [y0]   # [m]   Lista de posiciones Y en cada paso
        self.history_t: list = [0.0]  # [s]   Lista de tiempos correspondientes
        self._elapsed:  float = 0.0   # [s]   Tiempo total simulado acumulado

    def step(self, v: float, w: float):
        """
        Aplica una acción de control e integra la cinemática un paso dt.

        Entrada:
            v [m/s]   velocidad lineal   (salida del MPC)
            w [rad/s] velocidad angular  (salida del MPC)

        Actualiza: _x, _y, _theta, history_x/y/t, _elapsed
        """
        # Añade ruido gaussiano para simular imperfecciones físicas
        if self.noise:
            v_noisy: float = v + np.random.normal(0.0, _NOISE_POS / self.dt)
            w_noisy: float = w + np.random.normal(0.0, _NOISE_THETA / self.dt)
        else:
            v_noisy, w_noisy = v, w

        # Integración punto-medio (Runge-Kutta orden 2): más precisa que Euler simple
        theta_mid: float = self._theta + w_noisy * self.dt * 0.5  # heading interpolado
        self._x    += v_noisy * np.cos(theta_mid) * self.dt        # nueva posición X
        self._y    += v_noisy * np.sin(theta_mid) * self.dt        # nueva posición Y
        self._theta = _wrap(self._theta + w_noisy * self.dt)        # nuevo heading

        # Acumula historial para el plot final
        self._elapsed += self.dt
        self.history_x.append(self._x)
        self.history_y.append(self._y)
        self.history_t.append(self._elapsed)

    @property
    def state(self) -> np.ndarray:
        """
        Estado actual del robot simulado.

        Salida: np.ndarray shape (3,) → [X [m], Y [m], θ [rad]]
                Compatible con la interfaz de StateEstimator.state.
        """
        return np.array([self._x, self._y, self._theta])

    def reset(self, x: float, y: float, theta: float):
        """Reinicia el estado interno del robot sin limpiar el historial."""
        self._x, self._y, self._theta = x, y, theta


def _wrap(angle: float) -> float:
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


# ═══════════════════════════════════════════════════════════════
#  PRUEBA STANDALONE — ejecuta con: python simulation/sim_robot.py
# ═══════════════════════════════════════════════════════════════
if __name__ == '__main__':
    import sys, os
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

    import matplotlib.pyplot as plt
    from control.mpc_controller import MPCController
    from trajectory.lemniscata  import LemniscataTrajectory
    from config.robot_params    import (
        DT, MPC_HORIZON, Q, R,
        MAX_LINEAR_VEL, MAX_ANGULAR_VEL,
        LEMN_AMPLITUDE, LEMN_PERIOD
    )

    logging.basicConfig(level=logging.INFO,
                        format='%(asctime)s [%(levelname)s] %(message)s')

    # Componentes
    traj = LemniscataTrajectory(amplitude=LEMN_AMPLITUDE, period=LEMN_PERIOD)
    mpc  = MPCController(
        N     = MPC_HORIZON,
        dt    = DT,
        Q     = Q,
        R     = R,
        v_max = MAX_LINEAR_VEL,
        v_min = -MAX_LINEAR_VEL * 0.3,
        w_max = MAX_ANGULAR_VEL,
        w_min = -MAX_ANGULAR_VEL,
    )

    start = traj.start_pose
    robot = SimRobot(x0=start[0], y0=start[1], theta0=start[2],
                     dt=DT, noise=True)

    # Verifica que la velocidad máxima requerida sea alcanzable
    v_req = traj.max_speed
    log.info("Velocidad máxima requerida: %.3f m/s (máx robot: %.3f m/s)",
             v_req, MAX_LINEAR_VEL)
    if v_req > MAX_LINEAR_VEL * 1.1:
        log.warning("¡La trayectoria requiere más velocidad de la disponible! "
                    "Reduce LEMN_AMPLITUDE o aumenta LEMN_PERIOD.")

    # Bucle de simulación
    t_total = LEMN_PERIOD * 1.5   # 1.5 vueltas
    n_steps = int(t_total / DT)
    errors  = []

    log.info("Simulando %d pasos (%.1f s)...", n_steps, t_total)
    t0 = time.monotonic()

    for k in range(n_steps):
        t_now = k * DT

        x_ref   = traj.reference_horizon(t_now, MPC_HORIZON, DT)
        v, w    = mpc.compute(robot.state, x_ref)
        robot.step(v, w)

        ref_now = traj.state_at(t_now)
        err     = np.hypot(robot.state[0] - ref_now[0],
                           robot.state[1] - ref_now[1])
        errors.append(err)

        if k % 40 == 0:
            log.info(
                "t=%5.1f s | pos=(%+.3f, %+.3f) | err=%.4f m | "
                "v=%+.3f ω=%+.3f | MPC: %.1f ms",
                t_now,
                robot.state[0], robot.state[1], err,
                v, w, mpc.solve_ms
            )

    elapsed = time.monotonic() - t0
    log.info(
        "Simulación: %.1f s → error medio=%.4f m, error máx=%.4f m",
        elapsed, np.mean(errors), np.max(errors)
    )

    # ── Plot ─────────────────────────────────────────────────
    fig, axes = plt.subplots(1, 2, figsize=(13, 5))
    fig.suptitle("MPC Robot — Lemniscata de Gerono (simulación)", fontsize=12)

    ax = axes[0]
    xs_ref, ys_ref = traj.full_path()
    ax.plot(xs_ref, ys_ref, '--', color='gray', lw=1.0, label='referencia')
    ax.plot(robot.history_x, robot.history_y, color='steelblue',
            lw=1.5, label='robot simulado')
    ax.plot(robot.history_x[0], robot.history_y[0],
            'go', ms=8, label='inicio')
    ax.set_aspect('equal')
    ax.set_xlabel('X [m]'); ax.set_ylabel('Y [m]')
    ax.set_title('Trayectoria XY')
    ax.legend(fontsize=9); ax.grid(True, alpha=0.3)

    ax2 = axes[1]
    t_axis = [k * DT for k in range(len(errors))]
    ax2.plot(t_axis, errors, color='tomato', lw=1.2)
    ax2.axhline(np.mean(errors), ls='--', color='gray', lw=0.8, label='media')
    ax2.set_xlabel('Tiempo [s]'); ax2.set_ylabel('Error posición [m]')
    ax2.set_title('Error de seguimiento')
    ax2.legend(fontsize=9); ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    #plt.savefig('sim_result.png', dpi=150)
    plt.show()
    #log.info("Plot guardado como sim_result.png")
