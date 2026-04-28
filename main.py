"""
main.py — Bucle de control MPC principal
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Raspberry Pi 4B | Python 3.11+

Flujo de arranque:
  1. Conectar UART, verificar comunicación
  2. [ESPERAR BOTÓN] → el usuario posiciona el robot y presiona
  3. Calibrar giroscopio (robot quieto 3 s)
  4. Bucle MPC a 20 Hz hasta nuevo pulso del botón o Ctrl+C

Flujo por ciclo (20 Hz):
  1. Leer frame de sensores (uart_handler → SensorFrame)
  2. Verificar .robot_running → si False: motores off, pause
  3. Actualizar pose estimada (state_estimator)
  4. Obtener horizonte de referencia (lemniscata)
  5. Resolver MPC → (v, ω)
  6. Convertir a PWM → enviar al ESP32

Modos:
  python main.py               → hardware real (UART serial)
  python main.py --sim         → simulación sin hardware
  python main.py --port /dev/ttyS0
"""
import argparse
import time
import logging
import signal
import sys
import math
import numpy as np

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
    datefmt='%H:%M:%S',
    stream=sys.stdout,
)
log = logging.getLogger('main')

from config.robot_params import (
    CONTROL_FREQ, DT, MPC_HORIZON, Q, R,
    MAX_LINEAR_VEL, MAX_ANGULAR_VEL,
    WHEEL_BASE, PWM_MAX, PWM_DEADBAND,
    LEMN_AMPLITUDE, LEMN_PERIOD,
)
from comm.uart_handler          import UARTHandler, STATUS_RUNNING
from estimation.state_estimator import StateEstimator
from trajectory.lemniscata      import LemniscataTrajectory
from control.mpc_controller     import MPCController

# ════════════════════════════════════════════════════════════════
#  CONVERSIÓN [v, ω] → [PWM_L, PWM_R]
# ════════════════════════════════════════════════════════════════
def velocity_to_pwm(v: float, w: float) -> tuple:
    L   = WHEEL_BASE
    v_L = v - w * L / 2.0
    v_R = v + w * L / 2.0

    def to_pwm(vel: float) -> int:
        if abs(vel) < 1e-4:
            return 0
        ratio = abs(vel) / MAX_LINEAR_VEL
        raw   = PWM_DEADBAND + ratio * (PWM_MAX - PWM_DEADBAND)
        return int(np.clip(raw, 0, PWM_MAX) * math.copysign(1, vel))

    return to_pwm(v_L), to_pwm(v_R)


# ════════════════════════════════════════════════════════════════
#  CONTROLADOR PRINCIPAL
# ════════════════════════════════════════════════════════════════
class RobotController:

    def __init__(self, uart_port: str, sim: bool):
        self.sim = sim

        self.traj = LemniscataTrajectory(
            amplitude=LEMN_AMPLITUDE,
            period=LEMN_PERIOD
        )
        self.mpc = MPCController(
            N=MPC_HORIZON, dt=DT, Q=Q, R=R,
            v_max=MAX_LINEAR_VEL,
            v_min=-MAX_LINEAR_VEL * 0.3,
            w_max=MAX_ANGULAR_VEL,
            w_min=-MAX_ANGULAR_VEL,
        )

        if sim:
            # Modo simulación: no hay UART ni botón físico
            from simulation.sim_robot import SimRobot
            start = self.traj.start_pose
            self._sim      = SimRobot(x0=start[0], y0=start[1], theta0=start[2], dt=DT)
            self.uart      = None
            self.estimator = StateEstimator(alpha=0.98)
        else:
            # Modo real: UARTHandler con callbacks de transición de estado
            self.estimator = StateEstimator(alpha=0.98)
            self.uart = UARTHandler(
                port      = uart_port,
                on_arm    = self._on_robot_armed,
                on_disarm = self._on_robot_disarmed,
            )

        self._running       = False
        self._mpc_enabled   = False   # False hasta que el botón sea presionado
        signal.signal(signal.SIGINT,  self._on_exit)
        signal.signal(signal.SIGTERM, self._on_exit)

    # ── Callbacks de transición del botón ─────────────────────

    def _on_robot_armed(self, frame):
        """Llamado cuando UART detecta IDLE→RUNNING (botón presionado)."""
        log.info("── ROBOT ARMADO ── Reiniciando estimador al origen (0,0)")
        # El ESP32 ya zeroed los encoders en el momento del botón.
        # Aquí reiniciamos la pose de la RPi al inicio de la trayectoria.
        start = self.traj.start_pose
        self.estimator.reset(x=start[0], y=start[1], theta=start[2])
        self._mpc_enabled = True

    def _on_robot_disarmed(self, frame):
        """Llamado cuando UART detecta RUNNING→IDLE (botón presionado de nuevo)."""
        log.info("── ROBOT DETENIDO ── MPC pausado")
        self._mpc_enabled = False
        if self.uart:
            self.uart.send_command(0, 0)

    # ── Ejecución principal ───────────────────────────────────

    def run(self):
        self._banner()

        if self.sim:
            self._run_simulation()
            return

        # ── Modo real ─────────────────────────────────────────
        self.uart.start()
        self._wait_first_frame()

        # Bloquea hasta que el usuario presione el botón físico
        armed = self.uart.wait_for_arm(timeout=300.0)
        if not armed:
            log.error("Sin armado tras 5 min. Abortando.")
            self._shutdown()
            return

        # Calibración de giroscopio (robot quieto después de armar)
        self._calibrate_gyro()

        self._running = True
        self._run_loop()

    def _run_loop(self):
        """Bucle de control a frecuencia fija."""
        loop_dt   = 1.0 / CONTROL_FREQ
        next_tick = time.monotonic() + loop_dt
        t0        = time.monotonic()
        iteration = 0

        log.info("Bucle de control activo @ %d Hz", CONTROL_FREQ)

        while self._running:
            t_now = time.monotonic() - t0

            frame = self.uart.get_latest()
            if frame is None:
                self._sleep_to(next_tick)
                next_tick += loop_dt
                continue

            # ── MPC pausado si el usuario detuvo con el botón ──
            if not frame.robot_running:
                # No enviar comandos: el ESP32 tampoco los ejecutaría,
                # pero es buena práctica no generar tráfico inútil.
                self._sleep_to(next_tick)
                next_tick += loop_dt
                continue

            # ── Ciclo de control normal ─────────────────────────
            state = self.estimator.update(frame)
            x_ref = self.traj.reference_horizon(t_now, MPC_HORIZON, DT)
            v_cmd, w_cmd = self.mpc.compute(state, x_ref)

            pwm_l, pwm_r = velocity_to_pwm(v_cmd, w_cmd)
            self.uart.send_command(pwm_l, pwm_r)

            if iteration % CONTROL_FREQ == 0:
                self._log_status(t_now, state, v_cmd, w_cmd)

            iteration += 1
            self._sleep_to(next_tick)
            next_tick += loop_dt

        self._shutdown()

    def _run_simulation(self):
        """Bucle completo en modo simulación (sin hardware ni botón)."""
        log.info("[SIM] Modo simulación — sin botón físico, arranca directo")
        start = self.traj.start_pose
        self.estimator.reset(x=start[0], y=start[1], theta=start[2])
        self._mpc_enabled = True
        self._running     = True

        t0 = time.monotonic()
        n_steps = int(LEMN_PERIOD * 1.5 / DT)

        for k in range(n_steps):
            if not self._running:
                break
            t_now = k * DT
            state = self._sim.state
            x_ref = self.traj.reference_horizon(t_now, MPC_HORIZON, DT)
            v_cmd, w_cmd = self.mpc.compute(state, x_ref)
            self._sim.step(v_cmd, w_cmd)

            if k % CONTROL_FREQ == 0:
                self._log_status(t_now, state, v_cmd, w_cmd)

        log.info("[SIM] Completado en %.1f s", time.monotonic() - t0)
        self._plot_simulation()

    # ── Métodos de soporte ─────────────────────────────────────

    def _wait_first_frame(self, timeout: float = 6.0):
        log.info("Verificando comunicación UART con el ESP32...")
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            if self.uart.get_latest() is not None:
                log.info("✓ Comunicación UART OK")
                return
            time.sleep(0.05)
        self._shutdown()
        raise RuntimeError("Sin frames en %.0f s. Verifica UART y firmware." % timeout)

    def _calibrate_gyro(self):
        log.info("Calibrando bias del giroscopio (mantén el robot inmóvil 3 s)...")
        while not self.estimator.bias_calibrated:
            frame = self.uart.get_latest()
            if frame:
                self.estimator.calibrate_gyro(frame)
            time.sleep(DT)
        log.info("✓ Calibración de giroscopio completa")

    def _log_status(self, t: float, state: np.ndarray, v: float, w: float):
        ref  = self.traj.state_at(t)
        err  = math.hypot(state[0] - ref[0], state[1] - ref[1])
        diag = self.mpc.diagnostics
        log.info(
            "t=%6.1f | x=%+.3f y=%+.3f θ=%+5.1f° | "
            "err=%.4f m | v=%+.3f ω=%+.4f | MPC %.1f ms",
            t, state[0], state[1], math.degrees(state[2]),
            err, v, w, diag['solve_ms']
        )

    def _plot_simulation(self):
        try:
            import matplotlib.pyplot as plt
            fig, ax = plt.subplots(1, 1, figsize=(7, 6))
            fig.suptitle("MPC Robot — Lemniscata de Gerono (simulación)")
            xs_ref, ys_ref = self.traj.full_path()
            ax.plot(xs_ref, ys_ref, '--', color='gray', lw=1, label='ref')
            ax.plot(self._sim.history_x, self._sim.history_y,
                         color='steelblue', lw=1.5, label='robot')
            ax.axhline(0, color='black', linewidth=0.8, linestyle='--')
            ax.axvline(0, color='black', linewidth=0.8, linestyle='--')
            ax.set_aspect('equal')
            ax.set_title('Trayectoria XY')
            ax.legend(); ax.grid(True, alpha=0.3)
            plt.tight_layout()
            # plt.savefig('sim_result.png', dpi=150)
            # log.info("Plot guardado: sim_result.png")
            plt.show()
        except ImportError:
            log.info("matplotlib no disponible — omitiendo plot")

    def _shutdown(self):
        if not self.sim and self.uart:
            self.uart.send_command(0, 0)
            time.sleep(0.15)
            self.uart.stop()
        log.info("Controlador detenido.")

    def _on_exit(self, *_):
        log.info("Señal de parada recibida — deteniendo...")
        self._running = False

    @staticmethod
    def _sleep_to(target: float):
        r = target - time.monotonic()
        if r > 0:
            time.sleep(r)

    @staticmethod
    def _banner():
        log.info("═" * 55)
        log.info("  Robot MPC — Lemniscata de Gerono")
        log.info("  N=%d  dt=%.3f s  %.0f Hz  A=%.2f m  T=%.1f s",
                 MPC_HORIZON, DT, CONTROL_FREQ, LEMN_AMPLITUDE, LEMN_PERIOD)
        log.info("═" * 55)

# ════════════════════════════════════════════════════════════════
if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--port', default='/dev/serial0')
    p.add_argument('--sim', action='store_true')
    args = p.parse_args()
    RobotController(uart_port=args.port, sim=args.sim).run()
