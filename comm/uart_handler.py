"""
comm/uart_handler.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Comunicación UART bidireccional con el ESP32-C6.

Protocolo sensor (ESP32 → RPi): 23 bytes big-endian
  [0xAA][P1:4][P2:4][AX:2][AY:2][AZ:2][GX:2][GY:2][GZ:2][ST:1][0x55]

  ST = 0x00  STATUS_IDLE    → MPC debe mantenerse pausado
  ST = 0x02  STATUS_RUNNING → MPC autorizado, encoders zeroed en origen

Protocolo comando (RPi → ESP32): 6 bytes big-endian
  [0xBB][PWM_L:2][PWM_R:2][0x66]   (int16 con signo, -1000..+1000)

Novedad vs versión anterior:
  - FRAME_SENSOR_BYTES = 23 (era 22)
  - SensorFrame incluye campo .status y .robot_running
  - UARTHandler expone .waiting_for_arm() para que main.py
    bloquee el MPC hasta que el usuario presione el botón físico
  - Detección automática de transición IDLE→RUNNING para
    notificar al StateEstimator que debe hacer reset de pose
"""
import serial
import struct
import threading
import time
import logging
from dataclasses import dataclass, field
from typing import Optional, Callable

log = logging.getLogger(__name__)

# ── Constantes de protocolo ─────────────────────────────────────
_SENSOR_SIZE:  int = 23      # Bytes totales del frame de sensores (incluyendo cabecera y fin)
_SENSOR_START: int = 0xAA    # Byte de inicio del frame de sensores
_SENSOR_END:   int = 0x55    # Byte de fin del frame de sensores
_CMD_SIZE:     int = 6       # Bytes totales del frame de comando PWM
_CMD_START:    int = 0xBB    # Byte de inicio del frame de comando
_CMD_END:      int = 0x66    # Byte de fin del frame de comando

# Valores del byte STATUS del firmware ESP32
STATUS_IDLE:    int = 0x00   # FSM en reposo — botón no presionado, MPC bloqueado
STATUS_RUNNING: int = 0x02   # FSM activa — botón presionado, encoders=0, MPC habilitado

# ── Factores de conversión MPU6050 (raw LSB → unidades físicas) ──
_ACCEL_SCALE: float = 16384.0               # [LSB/g]   Rango ±2g
_GYRO_SCALE:  float = 131.0                 # [LSB/°/s] Rango ±250°/s
_DEG2RAD:     float = 3.14159265358979 / 180.0  # Factor grados → radianes


@dataclass
class SensorFrame:
    """
    Datos de un frame del ESP32, ya convertidos a unidades físicas.

    El campo `status` refleja el estado de la FSM del firmware:
      - STATUS_IDLE    (0x00): usuario no ha presionado el botón
      - STATUS_RUNNING (0x02): botón presionado, encoders=0, MPC ok
    """
    enc1:      int    # [ticks] Acumulado encoder rueda izquierda (int32, con signo)
    enc2:      int    # [ticks] Acumulado encoder rueda derecha  (int32, con signo)
    ax:        float  # [g]     Aceleración eje X del IMU
    ay:        float  # [g]     Aceleración eje Y del IMU
    az:        float  # [g]     Aceleración eje Z del IMU (≈ 1g en reposo horizontal)
    gx:        float  # [rad/s] Velocidad angular eje X (roll rate)
    gy:        float  # [rad/s] Velocidad angular eje Y (pitch rate)
    gz:        float  # [rad/s] Velocidad angular eje Z (yaw rate) — usado para heading
    status:    int    # Byte de estado de la FSM: STATUS_IDLE (0x00) o STATUS_RUNNING (0x02)
    timestamp: float  = field(default_factory=time.monotonic)
                       # [s] Tiempo de recepción (monotónico, no se reinicia al reboot)

    @property
    def robot_running(self) -> bool:
        """True si el ESP32 está en STATE_RUNNING (botón presionado, MPC habilitado)."""
        return self.status == STATUS_RUNNING


class UARTHandler:
    """
    Manejador UART thread-safe para RPi ↔ ESP32.

    Incluye detección de transición IDLE→RUNNING para notificar
    al StateEstimator que debe reiniciar la pose a (0,0,θ_actual).

    Uso básico:
        uart = UARTHandler('/dev/serial0')
        uart.start()

        # Espera al botón físico antes de arrancar el MPC
        uart.wait_for_arm()

        while True:
            frame = uart.get_latest()
            if frame and frame.robot_running:
                # ejecutar MPC normalmente
                ...
            else:
                # usuario paró el robot con el botón
                ...

        uart.stop()
    """

    def __init__(
        self,
        port:      str                    = '/dev/serial0',  # Puerto serie del sistema
                                                              # (RPi: /dev/serial0 o /dev/ttyS0)
        baud:      int                    = 921600,           # Baudrate — debe coincidir con firmware
        on_arm:    Optional[Callable]     = None,             # Callback al detectar IDLE→RUNNING
                                                              # Firma: fn(frame: SensorFrame)
        on_disarm: Optional[Callable]     = None,             # Callback al detectar RUNNING→IDLE
                                                              # Firma: fn(frame: SensorFrame)
    ):
        # Puerto serie bloqueante con timeout corto (no bloquea el hilo lector)
        self._ser = serial.Serial(
            port, baud,
            bytesize = serial.EIGHTBITS,
            parity   = serial.PARITY_NONE,
            stopbits = serial.STOPBITS_ONE,
            timeout  = 0.05,   # [s] timeout de lectura — evita bloqueo indefinido
        )

        self._lock:   threading.Lock              = threading.Lock()
        self._latest: Optional[SensorFrame]       = None   # Último frame recibido (thread-safe)
        self._n_ok:   int                          = 0      # Contador de frames válidos recibidos
        self._n_err:  int                          = 0      # Contador de frames con error/descartados

        # Estado anterior para detectar transiciones de la FSM del ESP32
        self._prev_status: Optional[int] = None   # STATUS del frame anterior (None = primer frame)

        # Callbacks de transición de estado (opcionales)
        self._on_arm:    Optional[Callable] = on_arm     # Llamado en IDLE→RUNNING
        self._on_disarm: Optional[Callable] = on_disarm  # Llamado en RUNNING→IDLE

        # Evento para bloquear main.py hasta que el botón sea presionado
        self._armed_event: threading.Event = threading.Event()

        self._running: bool             = False   # Flag de control del hilo lector
        self._thread:  threading.Thread = threading.Thread(
            target = self._reader_loop,
            name   = 'uart-reader',
            daemon = True,   # El hilo muere automáticamente al cerrar el proceso principal
        )

    # ── API pública ────────────────────────────────────────────

    def start(self):
        """Inicia el hilo lector de UART en segundo plano."""
        self._running = True
        self._thread.start()
        log.info("UART abierto: %s @ %d baud", self._ser.port, self._ser.baudrate)

    def stop(self):
        """Detiene el hilo lector y cierra el puerto serie."""
        self._running = False
        self._thread.join(timeout=2.0)
        if self._ser.is_open:
            self._ser.close()
        log.info("UART cerrado. Frames OK:%d ERR:%d", self._n_ok, self._n_err)

    def get_latest(self) -> Optional[SensorFrame]:
        """
        Retorna el último SensorFrame recibido (thread-safe, no bloqueante).

        Salida: SensorFrame o None si aún no se ha recibido ningún frame.
        """
        with self._lock:
            return self._latest

    def send_command(self, pwm_left: int, pwm_right: int):
        """
        Envía un comando PWM al ESP32 via UART.

        El ESP32 ignora el comando en STATE_IDLE (seguridad en firmware).

        Entrada:
            pwm_left  [-1000, 1000]  PWM para rueda izquierda (signo = dirección)
            pwm_right [-1000, 1000]  PWM para rueda derecha   (signo = dirección)
        """
        # Clampea a rango válido antes de empaquetar
        pwm_l: int = int(max(-1000, min(1000, pwm_left)))
        pwm_r: int = int(max(-1000, min(1000, pwm_right)))

        # Empaqueta manualmente en big-endian con cabecera y fin de trama
        buf = bytes([
            _CMD_START,
            (pwm_l >> 8) & 0xFF, pwm_l & 0xFF,    # PWM_L int16 big-endian
            (pwm_r >> 8) & 0xFF, pwm_r & 0xFF,    # PWM_R int16 big-endian
            _CMD_END,
        ])
        try:
            with self._lock:
                self._ser.write(buf)
        except serial.SerialException as e:
            log.error("Error enviando comando: %s", e)

    def wait_for_arm(self, timeout: float = 300.0) -> bool:
        """
        Bloquea hasta que el usuario presione el botón físico en el ESP32.

        Entrada:  timeout [s] — tiempo máximo de espera (por defecto 5 min)
        Salida:   True si el botón fue presionado antes del timeout,
                  False si expiró sin acción del usuario.
        """
        log.info("Esperando botón físico de armado en el ESP32...")
        log.info("   (Presiona el pulsador para definir el origen 0,0)")
        armed: bool = self._armed_event.wait(timeout=timeout)
        if armed:
            log.info("Robot ARMADO — MPC autorizado")
        else:
            log.warning("Timeout esperando botón (%.0f s)", timeout)
        return armed

    @property
    def is_armed(self) -> bool:
        """True si el último frame recibido indica STATE_RUNNING."""
        with self._lock:
            f = self._latest
        return f is not None and f.robot_running

    @property
    def stats(self) -> dict:
        """Devuelve contadores de frames válidos y erróneos desde el inicio."""
        return {'ok': self._n_ok, 'errors': self._n_err}

    # ── Hilo lector ────────────────────────────────────────────

    def _reader_loop(self):
        """
        Bucle principal del hilo lector UART.
        Lee frames continuamente mientras self._running sea True.
        """
        while self._running:
            try:
                frame = self._read_one_frame()   # intenta leer un frame completo
                if frame is not None:
                    self._handle_frame(frame)    # almacena y detecta transiciones
            except serial.SerialException as e:
                log.error("SerialException: %s", e)
                time.sleep(0.1)   # pausa antes de reintentar para no saturar el log
            except Exception as e:
                self._n_err += 1
                log.debug("Frame descartado: %s", e)

    def _handle_frame(self, frame: SensorFrame):
        """
        Almacena el frame recibido y detecta transiciones de estado FSM.

        Entrada: frame — SensorFrame válido recién recibido
        Efectos: actualiza _latest, detecta IDLE↔RUNNING y llama callbacks.
        """
        with self._lock:
            prev = self._prev_status        # estado anterior (puede ser None)
            self._latest      = frame       # actualiza el frame disponible para main
            self._prev_status = frame.status
            self._n_ok       += 1

        # Transición IDLE → RUNNING: el usuario presionó el botón
        if prev == STATUS_IDLE and frame.status == STATUS_RUNNING:
            log.info("[UART] Transición IDLE→RUNNING detectada (botón presionado)")
            self._armed_event.set()    # desbloquea wait_for_arm()
            if self._on_arm:
                self._on_arm(frame)    # notifica al StateEstimator para reset de pose

        # Transición RUNNING → IDLE: el usuario detuvo el robot
        elif prev == STATUS_RUNNING and frame.status == STATUS_IDLE:
            log.info("[UART] Transición RUNNING→IDLE detectada (robot detenido)")
            self._armed_event.clear()  # vuelve a bloquear para una próxima espera
            if self._on_disarm:
                self._on_disarm(frame)

    def _read_one_frame(self) -> Optional[SensorFrame]:
        """
        Sincroniza en el byte de inicio (0xAA) y lee un frame completo de sensores.

        Salida: SensorFrame si el frame es válido, None si no hay datos o hay error.
        """
        b = self._ser.read(1)   # lee 1 byte buscando el inicio de trama
        if not b or b[0] != _SENSOR_START:
            return None   # no es cabecera o timeout — el caller reintentará

        # Lee los bytes restantes del frame (total - 1 cabecera ya consumida)
        rest = self._ser.read(_SENSOR_SIZE - 1)
        if len(rest) != _SENSOR_SIZE - 1:
            return None   # frame incompleto (timeout de UART)

        if rest[-1] != _SENSOR_END:
            self._n_err += 1
            return None   # byte de fin incorrecto — frame corrompido

        # Layout de `rest` (22 bytes tras el 0xAA):
        # [P1:4][P2:4][AX:2][AY:2][AZ:2][GX:2][GY:2][GZ:2][ST:1][END:1]
        # Desempaqueta: 2×int32 + 6×int16 = 20 bytes, luego 1 byte de status
        p1, p2, ax_r, ay_r, az_r, gx_r, gy_r, gz_r = struct.unpack(
            '>2i6h', rest[:20]
        )
        status: int = rest[20]   # byte 21 del frame (posición 0-indexada en `rest`)

        return SensorFrame(
            enc1      = p1,
            enc2      = p2,
            ax        = ax_r / _ACCEL_SCALE,               # [g]
            ay        = ay_r / _ACCEL_SCALE,               # [g]
            az        = az_r / _ACCEL_SCALE,               # [g]
            gx        = gx_r / _GYRO_SCALE * _DEG2RAD,    # [rad/s]
            gy        = gy_r / _GYRO_SCALE * _DEG2RAD,    # [rad/s]
            gz        = gz_r / _GYRO_SCALE * _DEG2RAD,    # [rad/s]
            status    = status,
            timestamp = time.monotonic(),                  # [s] marca de recepción
        )
