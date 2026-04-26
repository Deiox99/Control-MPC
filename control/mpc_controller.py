"""
control/mpc_controller.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Model Predictive Controller para robot diferencial (unicycle).

Modelo cinemático (Euler discreto, dt fijo):
    X_{k+1} = X_k + v_k · cos(θ_k) · dt
    Y_{k+1} = Y_k + v_k · sin(θ_k) · dt
    θ_{k+1} = wrap(θ_k + ω_k · dt)

Problema de optimización:
    min   Σ_{i=1}^{N} [ ẽᵢᵀ Q ẽᵢ + uᵢᵀ R uᵢ ]
    s.a.  v_min ≤ v_k ≤ v_max   ∀k
          ω_min ≤ ω_k ≤ ω_max   ∀k

Donde ẽᵢ = [Δx, Δy, wrap(Δθ)] es el error de estado.
Solucionado con SLSQP (scipy) — eficiente en RPi 4B a 20 Hz.

Guía de tuning rápida:
  Q_x, Q_y ↑  → sigue posición más rápido (puede oscilar si es muy alto)
  Q_θ      ↑  → corrige heading antes de seguir posición
  R_v, R_ω ↑  → movimientos más suaves, respuesta más lenta
  N        ↑  → mejor anticipación, más tiempo de cómputo
"""
import numpy as np
from scipy.optimize import minimize, Bounds
import time
import logging

log = logging.getLogger(__name__)


class MPCController:
    """
    MPC no-lineal basado en SLSQP con warm-start y watch-dog.

    El warm-start reutiliza la solución anterior desplazada un paso,
    reduciendo el número de iteraciones del optimizador ~40%.
    """

    def __init__(
        self,
        N:     int         = 10,    # Horizonte de predicción [pasos].
                                    # Entrada: entero positivo. Más alto = mejor
                                    # anticipación pero más cómputo.
        dt:    float       = 0.05,  # Período de muestreo [s]. Debe coincidir con
                                    # el DT del bucle de control principal.
        Q:     np.ndarray  = None,  # Matriz de pesos de estado 3×3 [x, y, θ].
                                    # Diagonal. Penaliza error de seguimiento.
        R:     np.ndarray  = None,  # Matriz de pesos de control 2×2 [v, ω].
                                    # Diagonal. Penaliza esfuerzo de actuación.
        v_max: float       = 0.20,  # [m/s]   Límite superior de velocidad lineal.
        v_min: float       = -0.08, # [m/s]   Límite inferior (marcha atrás limitada).
        w_max: float       = 2.0,   # [rad/s] Límite superior de velocidad angular.
        w_min: float       = -2.0,  # [rad/s] Límite inferior de velocidad angular.
    ):
        self.N  = N                                          # Horizonte [pasos]
        self.dt = dt                                         # Período de muestreo [s]
        self.Q  = Q if Q is not None else np.diag([20.0, 20.0, 5.0])
        self.R  = R if R is not None else np.diag([0.5,  1.0])

        # Tamaño del vector de control aplanado: 2 entradas × N pasos
        self._n_ctrl: int = N * 2

        # Límites de control aplanados para scipy.optimize.Bounds
        # lb/ub shape: (N*2,) con pares [v_min, w_min] repetidos N veces
        lb = np.tile([v_min, w_min], N)   # vector de límites inferiores
        ub = np.tile([v_max, w_max], N)   # vector de límites superiores
        self._bounds = Bounds(lb, ub)

        # Vector de warm-start: secuencia de control del ciclo anterior [v0,ω0,v1,ω1,...]
        # Inicializado a cero (primer ciclo sin información previa).
        self._u_warm: np.ndarray = np.zeros(self._n_ctrl)

        # ── Diagnósticos (actualizados tras cada llamada a compute) ──
        self.solve_ms:  float = 0.0    # [ms]  Tiempo de cómputo del último solve
        self.last_cost: float = 0.0    # Valor de la función de costo en la solución
        self.last_ok:   bool  = True   # True si el solver convergió con éxito

    # ── API pública ────────────────────────────────────────────

    def compute(
        self,
        x0:    np.ndarray,   # Estado actual del robot: [X [m], Y [m], θ [rad]]
        x_ref: np.ndarray,   # Horizonte de referencia shape (N, 3): [x_ref, y_ref, θ_ref]
    ) -> tuple:
        """
        Resuelve el MPC y retorna la primera acción de control óptima.

        Entrada:
            x0:    estado actual         shape (3,) → [X, Y, θ]
            x_ref: referencia futura     shape (N, 3) → N × [x, y, θ]

        Salida:
            (v, ω): velocidad lineal [m/s] y angular [rad/s] para el paso k=0
        """
        t0 = time.perf_counter()   # marca de tiempo para medir duración del solve

        # Warm-start: reutiliza la solución anterior desplazada un paso
        u_init = self._shift_warm_start()  # shape (N*2,)

        result = minimize(
            fun     = self._cost,          # función objetivo a minimizar
            x0      = u_init,              # punto inicial (warm-start)
            args    = (x0, x_ref),         # argumentos adicionales a _cost
            method  = 'SLSQP',             # Sequential Least Squares Programming
            bounds  = self._bounds,        # restricciones de caja [v,ω]_min/max
            options = {
                'maxiter': 80,    # iteraciones máximas del optimizador
                'ftol':    1e-5,  # tolerancia de convergencia en la función objetivo
                'disp':    False, # no imprimir mensajes internos del solver
            }
        )

        # Actualiza diagnósticos
        self.solve_ms  = (time.perf_counter() - t0) * 1000.0
        self.last_cost = float(result.fun)
        self.last_ok   = result.success

        # Almacena solución para warm-start del próximo ciclo
        self._u_warm = result.x.copy()

        # SLSQP status=8 (incremento de gradiente positivo) es aceptable en práctica
        if not result.success and result.status not in (0, 8):
            log.warning(
                "MPC solver: %s (status=%d, cost=%.4f)",
                result.message, result.status, result.fun
            )

        # Extrae primera acción de control [v₀, ω₀] del vector optimizado
        v:  float = float(self._u_warm[0])   # [m/s]   velocidad lineal a aplicar
        om: float = float(self._u_warm[1])   # [rad/s] velocidad angular a aplicar
        return v, om

    @property
    def diagnostics(self) -> dict:
        """Devuelve métricas del último solve: tiempo, costo y convergencia."""
        return {
            'solve_ms':  round(self.solve_ms, 2),   # [ms]  duración del solve
            'cost':      round(self.last_cost, 4),   # valor del costo óptimo
            'converged': self.last_ok,               # True si convergió
        }

    # ── Funciones internas ─────────────────────────────────────

    def _cost(
        self,
        u_flat: np.ndarray,   # Vector de control aplanado shape (N*2,): [v0,ω0,v1,ω1,...]
        x0:     np.ndarray,   # Estado inicial para la propagación: [X, Y, θ]
        x_ref:  np.ndarray,   # Referencia del horizonte shape (N, 3)
    ) -> float:
        """
        Función de costo cuadrático del MPC.
        Evaluada repetidamente por el optimizador SLSQP.

        Costo = Σ_{i=0}^{N-1} [ eᵢᵀ Q eᵢ + uᵢᵀ R uᵢ ]
          donde eᵢ = estado_predicho[i] - x_ref[i]  (con wrap en θ)

        Salida: escalar J ≥ 0 (costo total del horizonte)
        """
        u: np.ndarray = u_flat.reshape(self.N, 2)   # (N, [v, ω]) — reorganiza el vector
        x: np.ndarray = x0.copy()                   # estado inicial para propagación
        J: float      = 0.0                          # acumulador de costo

        for i in range(self.N):
            # Propaga el modelo unicycle un paso con la entrada u[i]
            x = _unicycle_step(x, u[i, 0], u[i, 1], self.dt)

            # Error de estado: diferencia entre estado predicho y referencia
            e: np.ndarray = x - x_ref[i]
            e[2]          = _wrap(e[2])   # normaliza el error angular a (-π, π]

            # Costo cuadrático: error ponderado por Q + esfuerzo ponderado por R
            J += float(e @ self.Q @ e + u[i] @ self.R @ u[i])

        return J

    def _shift_warm_start(self) -> np.ndarray:
        """
        Desplaza la secuencia de control un paso adelante para warm-start.

        Transforma [v0,ω0, v1,ω1, ..., vN-1,ωN-1]
               en  [v1,ω1, v2,ω2, ..., vN-1,ωN-1, 0,0]

        Salida: vector shape (N*2,) — punto inicial para el siguiente solve
        """
        u: np.ndarray = self._u_warm.copy()
        u[:-2] = u[2:]    # desplaza todos los pares [v,ω] un paso hacia adelante
        u[-2:] = 0.0      # el último par no tiene información previa → inicializa a 0
        return u


# ── Funciones auxiliares (módulo-nivel para evitar overhead de método) ──

def _unicycle_step(
    x:  np.ndarray,   # Estado actual [X [m], Y [m], θ [rad]]
    v:  float,        # Velocidad lineal a aplicar [m/s]
    w:  float,        # Velocidad angular a aplicar [rad/s]
    dt: float,        # Período de integración [s]
) -> np.ndarray:
    """
    Integración Euler explícita del modelo cinemático unicycle.

    Ecuaciones:
        X_new = X + v·cos(θ)·dt
        Y_new = Y + v·sin(θ)·dt
        θ_new = wrap(θ + ω·dt)

    Salida: nuevo estado [X, Y, θ] como array numpy (no muta la entrada)
    """
    return np.array([
        x[0] + v * np.cos(x[2]) * dt,   # X_{k+1}
        x[1] + v * np.sin(x[2]) * dt,   # Y_{k+1}
        _wrap(x[2] + w * dt),            # θ_{k+1} normalizado
    ])


def _wrap(angle: float) -> float:
    """
    Normaliza un ángulo al rango (-π, π].

    Entrada:  ángulo en radianes (cualquier valor real)
    Salida:   ángulo equivalente en (-π, π]
    """
    return (angle + np.pi) % (2.0 * np.pi) - np.pi
