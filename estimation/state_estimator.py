"""
estimation/state_estimator.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Estimación de pose del robot diferencial.

Estado: x = [X, Y, θ]  en metros y radianes

Fusión:
  - Odometría de encoders → posición XY + heading (buena a corto plazo)
  - Giroscopio MPU6050   → tasa de giro gz (no deriva en posición)
  - Filtro complementario: θ = α·(θ + gz·dt) + (1-α)·θ_encoders

Por qué no Kalman aquí:
  - Para un prototipo a 20 Hz en RPi el filtro complementario
    tiene latencia mínima y es predecible.
  - Puedes reemplazar update() por un EKF sin cambiar el resto.
"""
import numpy as np
import logging
from typing import Optional
from comm.uart_handler import SensorFrame
from config.robot_params import METERS_PER_TICK, WHEEL_BASE

log = logging.getLogger(__name__)


class StateEstimator:
    """
    Estimador de pose para robot diferencial.

    alpha: peso del giroscopio en el filtro complementario.
      - alpha=1.0 → solo giroscopio (deriva con el tiempo)
      - alpha=0.0 → solo encoders  (sensible a deslizamiento)
      - alpha=0.98 es un buen punto de partida
    """

    def __init__(self, alpha: float = 0.98):
        # ── Parámetro del filtro complementario ──────────────────
        self.alpha: float = alpha   # Peso del giroscopio en la fusión de heading.
                                    # Rango (0, 1). Valor típico: 0.95 – 0.99.

        # ── Estado estimado (pose 2D del robot) ──────────────────
        self.x:     float = 0.0    # [m]   Posición en el eje X del mundo.
        self.y:     float = 0.0    # [m]   Posición en el eje Y del mundo.
        self.theta: float = 0.0    # [rad] Orientación (heading) del robot.
                                   #        Rango (-π, π]. 0 = mirando en +X.

        # ── Memoria del paso anterior (para calcular deltas) ─────
        self._prev_enc1: Optional[int]   = None  # Ticks acumulados encoder izq. (k-1)
        self._prev_enc2: Optional[int]   = None  # Ticks acumulados encoder der. (k-1)
        self._prev_time: Optional[float] = None  # Timestamp del frame anterior [s]

        # ── Calibración del bias del giroscopio ──────────────────
        self._gz_bias:        float = 0.0   # [rad/s] Offset DC estimado del giroscopio.
                                             #          Se resta en cada update.
        self._gz_samples:     list  = []     # Buffer de muestras gz en reposo para bias.
        self._bias_ready:     bool  = False  # True cuando el bias ya fue calculado.
        self._n_bias_samples: int   = 60     # Nº de muestras para promediar el bias.
                                             # A 20 Hz → ~3 s con el robot quieto.

    # ── API pública ────────────────────────────────────────────

    def reset(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0):
        """
        Reinicia la pose estimada a los valores indicados.

        Entrada:
            x     [m]   nueva posición X
            y     [m]   nueva posición Y
            theta [rad] nueva orientación
        """
        self.x, self.y, self.theta = x, y, theta
        self._prev_enc1 = None   # Fuerza re-inicialización en el próximo update
        self._prev_enc2 = None
        self._prev_time = None
        log.info(
            "Estado reiniciado → X=%.3f m, Y=%.3f m, θ=%.1f°",
            x, y, np.degrees(theta)
        )

    def calibrate_gyro(self, frame: SensorFrame) -> bool:
        """
        Acumula muestras de gz en reposo para estimar el bias del giroscopio.
        Llamar repetidamente durante los primeros N_BIAS_SAMPLES frames con el robot quieto.

        Entrada:  frame — SensorFrame con el campo .gz (velocidad angular Z) [rad/s]
        Salida:   True cuando la calibración completó, False mientras sigue acumulando.
        """
        if self._bias_ready:
            return True   # ya calibrado, no hace nada

        self._gz_samples.append(frame.gz)   # acumula muestra de giro en reposo

        if len(self._gz_samples) >= self._n_bias_samples:
            self._gz_bias   = float(np.mean(self._gz_samples))  # promedio = bias DC
            self._bias_ready = True
            log.info(
                "Bias giroscopio calibrado: %.4f rad/s (%.3f°/s)",
                self._gz_bias, np.degrees(self._gz_bias)
            )
        return self._bias_ready

    def update(self, frame: SensorFrame) -> np.ndarray:
        """
        Actualiza la pose a partir de un nuevo SensorFrame.

        Entrada:  frame — datos del ESP32 con encoders, IMU y timestamp.
        Salida:   np.ndarray shape (3,) → [X [m], Y [m], θ [rad]]
        """
        now: float = frame.timestamp   # [s] timestamp monotónico del frame actual

        # Primer frame: solo inicializa referencias, no integra
        if self._prev_enc1 is None:
            self._prev_enc1 = frame.enc1
            self._prev_enc2 = frame.enc2
            self._prev_time = now
            return self.state

        dt: float = now - self._prev_time   # [s] tiempo transcurrido desde último frame
        if dt <= 1e-6:                       # protección contra frames duplicados
            return self.state

        # ── Odometría de encoders ──────────────────────────────
        d_enc1: float = (frame.enc1 - self._prev_enc1) * METERS_PER_TICK  # [m] izquierda
        d_enc2: float = (frame.enc2 - self._prev_enc2) * METERS_PER_TICK  # [m] derecha

        # Actualiza memoria para el próximo ciclo
        self._prev_enc1 = frame.enc1
        self._prev_enc2 = frame.enc2
        self._prev_time = now

        d_center:   float = (d_enc1 + d_enc2) * 0.5       # [m]   avance lineal del centro
        dtheta_enc: float = (d_enc2 - d_enc1) / WHEEL_BASE # [rad] giro estimado por encoders

        # ── Giroscopio (eje Z = yaw) ───────────────────────────
        gz_corrected: float = frame.gz - self._gz_bias   # [rad/s] gz sin bias DC
        dtheta_gyro:  float = gz_corrected * dt           # [rad]   giro integrado

        # ── Filtro complementario para heading ─────────────────
        # alpha pondera el giroscopio (más preciso a corto plazo)
        # (1-alpha) pondera los encoders (no deriva a largo plazo)
        dtheta: float = self.alpha * dtheta_gyro + (1.0 - self.alpha) * dtheta_enc

        # ── Integración de posición (regla del punto medio) ────
        # Usa el ángulo a mitad del paso para mejor precisión (Runge-Kutta 2)
        theta_mid: float = self.theta + dtheta * 0.5   # [rad] heading interpolado
        self.x    += d_center * np.cos(theta_mid)       # [m]   nueva posición X
        self.y    += d_center * np.sin(theta_mid)       # [m]   nueva posición Y
        self.theta = _wrap(self.theta + dtheta)          # [rad] nuevo heading normalizado

        return self.state

    @property
    def state(self) -> np.ndarray:
        """
        Estado actual como array numpy.

        Salida: np.ndarray shape (3,) → [X [m], Y [m], θ [rad]]
        """
        return np.array([self.x, self.y, self.theta])

    @property
    def bias_calibrated(self) -> bool:
        """True si el bias del giroscopio ya fue estimado y está listo para usar."""
        return self._bias_ready


def _wrap(angle: float) -> float:
    """
    Normaliza un ángulo al rango (-π, π].

    Entrada:  ángulo en radianes (cualquier valor real)
    Salida:   ángulo equivalente en (-π, π]
    """
    return (angle + np.pi) % (2.0 * np.pi) - np.pi
