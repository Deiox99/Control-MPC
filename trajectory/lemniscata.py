"""
trajectory/lemniscata.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Lemniscata de Gerono — trayectoria de referencia para el MPC.

Ecuaciones paramétricas (τ = ω·t, ω = 2π/T):
    x(τ) = A · cos(τ)
    y(τ) = A · sin(τ) · cos(τ)  =  A/2 · sin(2τ)

Curva en forma de ∞ (figura 8). Al escalar a mariposa realista
después, solo necesitas cambiar la parametrización aquí.

El MPC pide reference_horizon(t_now, N, dt) que devuelve
los N estados futuros [x_ref, y_ref, theta_ref].
"""
import numpy as np


class LemniscataTrajectory:
    """
    Generador de referencia Lemniscata de Gerono.

    Parámetros:
        amplitude [m]: controla el tamaño del ∞
        period    [s]: tiempo para completar la figura completa
    """

    def __init__(self, amplitude: float = 0.30, period: float = 24.0):
        self.A:  float = amplitude          # [m]   Amplitud de cada lóbulo de la figura ∞.
        self.T:  float = period             # [s]   Período de una vuelta completa.
        self._w: float = 2.0 * np.pi / period  # [rad/s] Frecuencia angular ω = 2π/T.
                                                #          Velocidad de recorrido de la curva.

    def state_at(self, t: float) -> np.ndarray:
        """
        Retorna el estado de referencia [x, y, θ] en el instante t [s].

        Derivadas usadas para calcular el heading tangencial:
            dx/dt = -A·ω·sin(τ)
            dy/dt =  A·ω·cos(2τ)

        Entrada:  t [s] — tiempo de evaluación (puede ser cualquier valor real)
        Salida:   np.ndarray shape (3,) → [x [m], y [m], θ [rad]]
        """
        tau: float = self._w * t   # [rad] Fase de la curva en el instante t

        # Posición en la curva paramétrica
        x: float = self.A * np.cos(tau)                    # [m] coordenada X
        y: float = self.A * np.sin(tau) * np.cos(tau)     # [m] coordenada Y = A/2·sin(2τ)

        # Vector tangente a la curva (derivada temporal de la posición)
        dx: float = -self.A * self._w * np.sin(tau)        # [m/s] componente X del tangente
        dy: float =  self.A * self._w * np.cos(2.0 * tau)  # [m/s] componente Y del tangente

        # Heading tangencial: ángulo del vector velocidad respecto al eje X
        theta: float = np.arctan2(dy, dx)   # [rad] resultado en (-π, π]

        return np.array([x, y, theta])

    def reference_horizon(self, t_start: float, N: int, dt: float) -> np.ndarray:
        """
        Retorna los N estados de referencia futuros para el horizonte del MPC.

        Los waypoints corresponden a k+1, k+2, ..., k+N (no incluye el estado actual k),
        lo cual es la convención estándar en formulaciones MPC de control predictivo.

        Entrada:
            t_start [s]  — tiempo actual del bucle de control
            N        — número de pasos del horizonte (entero positivo)
            dt      [s]  — período de muestreo del MPC

        Salida: np.ndarray shape (N, 3) — cada fila es [x_ref [m], y_ref [m], θ_ref [rad]]
        """
        refs: np.ndarray = np.zeros((N, 3))
        for i in range(N):
            refs[i] = self.state_at(t_start + (i + 1) * dt)   # k+1 … k+N
        return refs

    def full_path(self, n_points: int = 400) -> tuple:
        """
        Genera la trayectoria completa para visualización (una vuelta entera).

        Entrada:  n_points — resolución del muestreo (más puntos = curva más suave)
        Salida:   (x_array, y_array) — dos arrays numpy de longitud n_points
        """
        t_vals: np.ndarray = np.linspace(0.0, self.T, n_points, endpoint=False)
        pts: np.ndarray    = np.array([self.state_at(t) for t in t_vals])
        return pts[:, 0], pts[:, 1]   # (xs [m], ys [m])

    @property
    def start_pose(self) -> np.ndarray:
        """
        Pose inicial de la trayectoria en t=0: [A, 0, θ_0].

        Salida: np.ndarray shape (3,) → [x [m], y [m], θ [rad]]
        """
        return self.state_at(0.0)

    # ── Métodos de validación ──────────────────────────────────

    def _speed_profile(self, t: float) -> float:
        """
        Velocidad lineal requerida en el instante t [m/s].

        Entrada: t [s]
        Salida:  |v(t)| = √(dx² + dy²) [m/s] — útil para verificar alcanzabilidad.
        """
        tau: float = self._w * t
        dx:  float = -self.A * self._w * np.sin(tau)
        dy:  float =  self.A * self._w * np.cos(2.0 * tau)
        return float(np.hypot(dx, dy))

    @property
    def max_speed(self) -> float:
        """
        Velocidad lineal máxima requerida a lo largo de la trayectoria [m/s].

        Salida: escalar [m/s] — comparar con MAX_LINEAR_VEL para verificar
                que el robot puede seguir la trayectoria.
        """
        t_vals: np.ndarray = np.linspace(0.0, self.T, 1000)
        return float(np.max([self._speed_profile(t) for t in t_vals]))
        return max(self._speed_profile(t) for t in t_vals)
