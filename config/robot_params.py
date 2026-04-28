"""
config/robot_params.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Único punto de verdad para parámetros físicos y de control.
CALIBRA estos valores con mediciones reales del robot físico.
Todos los valores en unidades SI (metros, radianes, segundos).
"""
import numpy as np

# ══════════════════════════════════════════════════
#  GEOMETRÍA DEL ROBOT
# ══════════════════════════════════════════════════
WHEEL_RADIUS: float = 0.03    # [m]  Radio de cada rueda motriz.
                               #      Medir con calibre en el punto de contacto.
WHEEL_BASE: float   = 0.25    # [m]  Distancia entre los dos puntos de contacto
                               #      de las ruedas con el suelo (track width).
                               #      NO es la distancia entre centros de motor.

# ══════════════════════════════════════════════════
#  ENCODER N20
# ══════════════════════════════════════════════════
# Motor N20 130 RPM: encoder de cuadratura de 7 pulsos/rev en eje de motor,
# caja reductora ~150:1  →  7 × 150 = 1050 ticks/rev de rueda (modo CHANGE).
# CALIBRACIÓN: girar la rueda 1 vuelta completa y contar los ticks reales.
ENCODER_PPR_MOTOR: int = 7     # Pulsos por revolución del eje del motor
                               # (valor del encoder magnético, no de rueda).
GEAR_RATIO: int        = 150   # Relación de reducción de la caja reductora.
                               # 150:1 → la rueda gira 1 vez por cada 150 del motor.
TICKS_PER_REV: int     = ENCODER_PPR_MOTOR * GEAR_RATIO
                               # Ticks totales por revolución completa de rueda.
                               # Entrada: PPR motor × relación caja.

METERS_PER_TICK: float = (2.0 * np.pi * WHEEL_RADIUS) / TICKS_PER_REV
                               # [m/tick] Distancia lineal recorrida por cada tick
                               # de encoder. Derivada de la circunferencia / ticks_rev.

# ══════════════════════════════════════════════════
#  MPU6050 — escalas por defecto (sin cambiar registros)
# ══════════════════════════════════════════════════
ACCEL_SCALE: float = 16384.0  # [LSB/g]   Factor de escala para acelerómetro.
                               # Rango ±2g → 2^15 / 2 = 16384 LSB por g.
GYRO_SCALE: float  = 131.0    # [LSB/°/s] Factor de escala para giroscopio.
                               # Rango ±250°/s → 2^15 / 250 ≈ 131 LSB por °/s.
DEG_TO_RAD: float  = np.pi / 180.0
                               # Factor de conversión de grados a radianes.

# ══════════════════════════════════════════════════
#  LÍMITES MECÁNICOS
# ══════════════════════════════════════════════════
MAX_RPM: float       = 130.0              # [RPM] Velocidad libre del motor N20 sin carga.
OPERATING_RPM: float = MAX_RPM * 0.70    # [RPM] RPM estimadas bajo carga (70% del libre).
                                          #       Ajustar según medición real con carga.

# Velocidad lineal máxima de la rueda: v = ω_rueda × R = (RPM × 2π/60) × R
MAX_WHEEL_VEL: float  = OPERATING_RPM * (2.0 * np.pi / 60.0) * WHEEL_RADIUS
                                          # [m/s] Velocidad periférica máxima de rueda.
MAX_LINEAR_VEL: float = MAX_WHEEL_VEL    # [m/s] Velocidad lineal máxima del robot
                                          #       (igual a MAX_WHEEL_VEL en diferencial).
MAX_ANGULAR_VEL: float = (2.0 * MAX_WHEEL_VEL) / WHEEL_BASE
                                          # [rad/s] Velocidad angular máxima del robot.
                                          # Derivada: ω_max = (v_R_max - v_L_min) / L
                                          # = 2·v_max / L  (giro en un punto).

# ══════════════════════════════════════════════════
#  MAPEO DE VELOCIDAD A PWM
# ══════════════════════════════════════════════════
PWM_MAX: int      = 1000   # Valor PWM máximo enviado al ESP32 (escala 0–1000).
PWM_DEADBAND: int = 80     # Valor mínimo de PWM para vencer fricción estática.
                            # CALIBRAR: incrementar hasta que el motor comience a girar.

# ══════════════════════════════════════════════════
#  TIMING DEL CONTROLADOR
# ══════════════════════════════════════════════════
CONTROL_FREQ: int  = 20           # [Hz]  Frecuencia del bucle de control MPC.
                                   #       50 ms por ciclo — limitado por tiempo de cómputo.
DT: float          = 1.0 / CONTROL_FREQ
                                   # [s]   Período de muestreo del controlador.
                                   #       Usado en integración cinemática y horizonte MPC.

# ══════════════════════════════════════════════════
#  PARÁMETROS MPC
# ══════════════════════════════════════════════════
MPC_HORIZON: int = 20    # [pasos] Horizonte de predicción.
                          # 20 pasos × 50 ms = 1.0 s de anticipación.
                          # Aumentar mejora seguimiento en curvas, incrementa cómputo.

# Matriz de pesos de estado Q [x, y, θ] — diagonal 3×3
# Q_x, Q_y altos → el robot sigue más agresivamente la posición (puede oscilar).
# Q_θ alto       → corrige el heading antes de avanzar hacia la posición.
Q: np.ndarray = np.diag([40.0, 40.0, 8.0])
                          # Entrada al MPCController.  Unidades: [1/m², 1/m², 1/rad²].

# Matriz de pesos de control R [v, ω] — diagonal 2×2
# R_v, R_ω altos → movimientos más suaves y lentos (menor esfuerzo de actuación).
R: np.ndarray = np.diag([0.5, 1.0])
                          # Entrada al MPCController.  Unidades: [s²/m², s²/rad²].

# ══════════════════════════════════════════════════
#  TRAYECTORIA — Lemniscata de Gerono
# ══════════════════════════════════════════════════
LEMN_AMPLITUDE: float = 1.00   # [m]  Amplitud de cada lóbulo de la figura ∞.
                                 #      X ∈ [-1, 1] m  |  Y ∈ [-0.5, 0.5] m
LEMN_PERIOD: float    = 36.0   # [s]  Tiempo para completar una figura ∞ completa.
                                 #      v_pico = A·(2π/T)·√2 ≈ 0.247 m/s < 0.286 m/s máx robot.
