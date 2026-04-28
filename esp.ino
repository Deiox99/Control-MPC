// ═══════════════════════════════════════════════════════════════
//  esp32_robot.ino — Robot MPC · nanoESP32-C6 v1.0
//  Driver    : DRV8833 (dual H-bridge, PWM en pines IN directamente)
//  Sensores  : 2× encoder N20 + MPU6050 (I2C)
//  Comms     : UART0 binary bidireccional con RPi @ 921600
//  Control   : FSM IDLE/RUNNING + botón físico de armado
//
//  ─── DIFERENCIA CLAVE vs L298N ──────────────────────────────
//  L298N:  IN1, IN2 (dirección)  +  ENA (PWM de velocidad)
//  DRV8833: NO hay pin ENA/ENB separado.
//           IN1 e IN2 son directamente las señales PWM.
//  
//  Tabla de control DRV8833 (por motor):
//    IN1=PWM  IN2=0      → avance   (duty=velocidad)
//    IN1=0    IN2=PWM    → retroceso
//    IN1=0    IN2=0      → libre (coast / inercia)
//    IN1=HIGH IN2=HIGH   → freno activo (cortocircuita bobinas)
//
//  Por eso los 4 pines IN reciben señal LEDC — no hay pin extra.
//
//  ─── PINOUT DRV8833 → nanoESP32-C6 ─────────────────────────
//  DRV8833    Función           GPIO ESP32-C6
//  ─────────  ───────────────── ───────────────────────────────
//  IN1        Motor L avance    GPIO 2
//  IN2        Motor L retroceso GPIO 3
//  IN3        Motor R avance    GPIO 4
//  IN4        Motor R retroceso GPIO 5
//  EEP        Enable driver     GPIO 8  (HIGH = activo)
//  OUT1/OUT2  Motor L           (al motor)
//  OUT3/OUT4  Motor R           (al motor)
//  VCC        Alimentación      3.3V o 5V del robot
//  GND        Tierra            GND común
//  ULT        Fault (opcional)  sin conectar o a GPIO libre
//
//  ─── Pines ya usados en el sistema ─────────────────────────
//  GPIO  1  → BTN (botón de armado)
//  GPIO  6  → SDA MPU6050
//  GPIO  7  → SCL MPU6050
//  GPIO 10  → ENC1_A (interrupción encoder izq)
//  GPIO 11  → ENC1_B (lectura encoder izq)
//  GPIO 16  → UART0 TX → RPi RX
//  GPIO 17  → UART0 RX ← RPi TX
//  GPIO 18  → ENC2_A (interrupción encoder der)
//  GPIO 19  → ENC2_B (lectura encoder der)
//
//  ─── Protocolo sensor (ESP32 → RPi): 23 bytes @ 100 Hz ─────
//  [0xAA][P1:4][P2:4][AX:2][AY:2][AZ:2][GX:2][GY:2][GZ:2][ST:1][0x55]
//  ST=0x00 IDLE | ST=0x02 RUNNING
//
//  ─── Protocolo comando (RPi → ESP32): 6 bytes ───────────────
//  [0xBB][PWM_L:2][PWM_R:2][0x66]  int16 en rango -1000..+1000
//
//  ─── Arduino IDE 2.3.8 / Core Espressif ─────────────────────
//  Core ≥ 3.0.0 : ledcAttach(pin, freq, bits)  ← este archivo
//  Core < 3.0.0 : ver nota al final
// ═══════════════════════════════════════════════════════════════

#include <Wire.h>

// ─── UART ─────────────────────────────────────────────────────
#define UART_BAUD           921600   // Serial0: GPIO16(TX) GPIO17(RX)

// ─── Encoders ─────────────────────────────────────────────────
#define ENC_L_A             10   // canal A motor izq (ISR CHANGE)
#define ENC_L_B             11   // canal B motor izq (solo lectura)
#define ENC_R_A             18   // canal A motor der (ISR CHANGE)
#define ENC_R_B             19   // canal B motor der (solo lectura)

// ─── MPU6050 ─────────────────────────────────────────────────
#define MPU_ADDR            0x68
#define SDA_PIN             6
#define SCL_PIN             7

// ─── DRV8833 — Motor L (IN1/IN2 → OUT1/OUT2) ─────────────────
// Ambos pines reciben señal LEDC-PWM directamente.
// IN1 controla el semiciclo positivo (avance).
// IN2 controla el semiciclo negativo (retroceso).
#define MOT_L_IN1           2    // → DRV8833 IN1
#define MOT_L_IN2           3    // → DRV8833 IN2

// ─── DRV8833 — Motor R (IN3/IN4 → OUT3/OUT4) ─────────────────
#define MOT_R_IN1           4    // → DRV8833 IN3
#define MOT_R_IN2           5    // → DRV8833 IN4

// ─── DRV8833 — Enable/Sleep ──────────────────────────────────
// EEP = HIGH → driver activo
// EEP = LOW  → driver en sleep (salidas en alta impedancia)
// Algunos módulos traen EEP ya en pullup interno a VCC.
// Si el tuyo lo trae: puedes ignorar este pin (comenta la línea).
// Si no: conéctalo a GPIO8 y el firmware lo activa en setup().
#define DRV_EEP             8    // Enable del DRV8833

// ─── LEDC PWM ─────────────────────────────────────────────────
// 20 kHz = ultrasónico, los N20 no producen zumbido audible.
// 10 bits de resolución → 0..1023 de duty cycle.
// Se adjunta a los 4 pines IN del DRV8833 (no hay pin PWM aparte).
#define PWM_FREQ_HZ         20000
#define PWM_BITS            10
#define PWM_MAX_DUTY        ((1 << PWM_BITS) - 1)   // 1023

// ─── Botón de armado ─────────────────────────────────────────
// Pulsador normalmente abierto entre GPIO1 y GND.
// INPUT_PULLUP: reposo = HIGH, presionado = LOW.
// ISR dispara en flanco FALLING (bajada = momento de presión).
#define BTN_PIN             1
#define DEBOUNCE_MS         60   // filtra rebotes mecánicos

// ─── Protocolo binario ───────────────────────────────────────
#define FRAME_SENSOR_BYTES  23
#define START_SENSOR        0xAA
#define END_SENSOR          0x55
#define FRAME_CMD_BYTES     6
#define START_CMD           0xBB
#define END_CMD             0x66
#define STATUS_IDLE         0x00
#define STATUS_RUNNING      0x02

// ─── Timing ──────────────────────────────────────────────────
#define SEND_MS             10   // 100 Hz de envío de sensores

// ═══════════════════════════════════════════════════════════════
//  FSM — Máquina de estados
// ═══════════════════════════════════════════════════════════════
typedef enum : uint8_t {
  STATE_IDLE    = STATUS_IDLE,      // 0x00
  STATE_RUNNING = STATUS_RUNNING    // 0x02
} robot_state_t;

volatile robot_state_t robot_state = STATE_IDLE;

// ═══════════════════════════════════════════════════════════════
//  VARIABLES GLOBALES
// ═══════════════════════════════════════════════════════════════
volatile int32_t  pos_L       = 0;    // ticks encoder izquierdo
volatile int32_t  pos_R       = 0;    // ticks encoder derecho
volatile uint32_t btn_last_ms = 0;    // debounce del botón

// ═══════════════════════════════════════════════════════════════
//  ISR — ENCODERS (cuadratura: 1 canal interrumpido, 1 leído)
// ═══════════════════════════════════════════════════════════════
void IRAM_ATTR isr_enc_L() {
  pos_L += (digitalRead(ENC_L_A) == digitalRead(ENC_L_B)) ? 1 : -1;
}
void IRAM_ATTR isr_enc_R() {
  pos_R += (digitalRead(ENC_R_A) == digitalRead(ENC_R_B)) ? 1 : -1;
}

// ═══════════════════════════════════════════════════════════════
//  ISR — BOTÓN DE ARMADO
// ═══════════════════════════════════════════════════════════════
void IRAM_ATTR isr_button() {
  uint32_t now = millis();
  if (now - btn_last_ms < DEBOUNCE_MS) return;   // rebote → ignorar
  btn_last_ms = now;

  if (robot_state == STATE_IDLE) {
    // Zerear encoders = definir el origen físico (0, 0)
    noInterrupts();
    pos_L = 0;
    pos_R = 0;
    interrupts();
    robot_state = STATE_RUNNING;
  } else {
    robot_state = STATE_IDLE;
    // Los motores se pararán en el próximo ciclo del loop()
  }
}

// ═══════════════════════════════════════════════════════════════
//  CONTROL DE MOTORES — DRV8833
//
//  val en rango -1000..+1000 (compatible con lo que envía la RPi).
//  Se escala internamente a 0..PWM_MAX_DUTY (1023 para 10 bits).
//
//  Modo avance   : in1=duty,  in2=0
//  Modo retroceso: in1=0,     in2=duty
//  Modo freno    : in1=1023,  in2=1023  (cortocircuito de bobinas)
//  Modo libre    : in1=0,     in2=0     (inercia / coast)
//
//  Se usa freno activo en val==0 para que los N20 con tornillo
//  se detengan instantáneamente y no deriven.
// ═══════════════════════════════════════════════════════════════
void drv_set(uint8_t in1, uint8_t in2, int16_t val) {
  // Escala de ±1000 a 0..1023
  int32_t duty = ((int32_t)abs(val) * PWM_MAX_DUTY) / 1000;
  if (duty > PWM_MAX_DUTY) duty = PWM_MAX_DUTY;

  if (val > 0) {
    // Avance: in1 lleva PWM, in2 en bajo
    ledcWrite(in1, (uint32_t)duty);
    ledcWrite(in2, 0);
  } else if (val < 0) {
    // Retroceso: in1 en bajo, in2 lleva PWM
    ledcWrite(in1, 0);
    ledcWrite(in2, (uint32_t)duty);
  } else {
    // Freno activo: ambos en alto → DRV8833 cortocircuita los terminales
    // Esto es especialmente útil con tornillo sin fin (inercial por default)
    ledcWrite(in1, PWM_MAX_DUTY);
    ledcWrite(in2, PWM_MAX_DUTY);
  }
}

void motors_stop() {
  drv_set(MOT_L_IN1, MOT_L_IN2, 0);   // freno activo motor L
  drv_set(MOT_R_IN1, MOT_R_IN2, 0);   // freno activo motor R
}

// ═══════════════════════════════════════════════════════════════
//  MPU6050 — lectura burst 14 bytes desde registro 0x3B
// ═══════════════════════════════════════════════════════════════
void readMPU(int16_t &ax, int16_t &ay, int16_t &az,
             int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14, (uint8_t)true);

  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();   // temperatura — descartar
  gx = (Wire.read() << 8) | Wire.read();
  gy = (Wire.read() << 8) | Wire.read();
  gz = (Wire.read() << 8) | Wire.read();
}

// ═══════════════════════════════════════════════════════════════
//  PARSER DE COMANDOS RPi → ESP32 (no bloqueante, gating por FSM)
//
//  En STATE_IDLE: el buffer se vacía sin ejecutar nada.
//    → Garantiza que comandos recibidos antes de armar NO se acumulen
//      y provoquen un arranque inesperado al presionar el botón.
//  En STATE_RUNNING: comandos válidos se aplican a los motores.
// ═══════════════════════════════════════════════════════════════
void parse_commands() {
  if (robot_state != STATE_RUNNING) {
    while (Serial0.available()) Serial0.read();   // vaciar sin ejecutar
    return;
  }

  while (Serial0.available() >= FRAME_CMD_BYTES) {
    if (Serial0.peek() != START_CMD) {
      Serial0.read();   // byte de basura → descartar
      continue;
    }
    uint8_t buf[FRAME_CMD_BYTES];
    int n = Serial0.readBytes(buf, FRAME_CMD_BYTES);
    if (n == FRAME_CMD_BYTES && buf[5] == END_CMD) {
      int16_t pwm_L = (int16_t)((buf[1] << 8) | buf[2]);
      int16_t pwm_R = (int16_t)((buf[3] << 8) | buf[4]);
      drv_set(MOT_L_IN1, MOT_L_IN2, pwm_L);
      drv_set(MOT_R_IN1, MOT_R_IN2, pwm_R);
    }
  }
}

// ═══════════════════════════════════════════════════════════════
//  ENVÍO DE FRAME DE SENSORES (23 bytes, big-endian)
//  [0xAA][P_L:4][P_R:4][AX:2][AY:2][AZ:2][GX:2][GY:2][GZ:2][ST:1][0x55]
// ═══════════════════════════════════════════════════════════════
void send_sensor_frame(uint8_t status_byte) {
  // Captura atómica de encoders
  noInterrupts();
  int32_t p_L = pos_L;
  int32_t p_R = pos_R;
  interrupts();

  int16_t ax, ay, az, gx, gy, gz;
  readMPU(ax, ay, az, gx, gy, gz);

  uint8_t buf[FRAME_SENSOR_BYTES];
  buf[0]  = START_SENSOR;
  // Encoder L — big-endian int32
  buf[1]  = (p_L >> 24) & 0xFF;
  buf[2]  = (p_L >> 16) & 0xFF;
  buf[3]  = (p_L >>  8) & 0xFF;
  buf[4]  =  p_L        & 0xFF;
  // Encoder R — big-endian int32
  buf[5]  = (p_R >> 24) & 0xFF;
  buf[6]  = (p_R >> 16) & 0xFF;
  buf[7]  = (p_R >>  8) & 0xFF;
  buf[8]  =  p_R        & 0xFF;
  // Acelerómetro — 3× int16
  buf[9]  = (ax >> 8) & 0xFF;  buf[10] = ax & 0xFF;
  buf[11] = (ay >> 8) & 0xFF;  buf[12] = ay & 0xFF;
  buf[13] = (az >> 8) & 0xFF;  buf[14] = az & 0xFF;
  // Giroscopio — 3× int16
  buf[15] = (gx >> 8) & 0xFF;  buf[16] = gx & 0xFF;
  buf[17] = (gy >> 8) & 0xFF;  buf[18] = gy & 0xFF;
  buf[19] = (gz >> 8) & 0xFF;  buf[20] = gz & 0xFF;
  // Estado FSM
  buf[21] = status_byte;
  buf[22] = END_SENSOR;

  Serial0.write(buf, FRAME_SENSOR_BYTES);

  // Debug USB-CDC — imprime 1 vez por segundo (cada 10 frames @ 100 Hz)
  static uint8_t dbg_cnt = 0;
  if (++dbg_cnt >= 10) {
    dbg_cnt = 0;
    const char* st = (status_byte == STATUS_RUNNING) ? "RUN " : "IDLE";
    Serial.printf("[%s] L:%7d R:%7d | GZ:%+7.2f deg/s\n",
                  st, p_L, p_R, gz / 131.0f);
  }
}

// ═══════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════
void setup() {
  // Puerto debug (USB-CDC, no interfiere con UART0 de la RPi)
  Serial.begin(115200);
  // UART0 bidireccional con RPi
  Serial0.begin(UART_BAUD, SERIAL_8N1);

  // ── MPU6050 ────────────────────────────────────────────────
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);   // Fast Mode 400 kHz

  // Despertar MPU (PWR_MGMT_1 = 0x00)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0x00);
  Wire.endTransmission(true);

  // Giroscopio: ±250°/s → escala 131 LSB/(°/s)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B); Wire.write(0x00);
  Wire.endTransmission(true);

  // Acelerómetro: ±2g → escala 16384 LSB/g
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C); Wire.write(0x00);
  Wire.endTransmission(true);

  // ── DRV8833 Enable ─────────────────────────────────────────
  // Activa el driver antes de configurar LEDC.
  // Si tu módulo ya tiene EEP en pullup interno, esta línea
  // no hace daño pero es redundante — puedes dejarla igual.
  pinMode(DRV_EEP, OUTPUT);
  digitalWrite(DRV_EEP, HIGH);   // HIGH = driver activo

  // ── LEDC PWM en los 4 pines IN del DRV8833 ─────────────────
  // Core 3.x API: ledcAttach(pin, freq_Hz, resolution_bits)
  // Esto reemplaza ledcSetup+ledcAttachPin de Core 2.x.
  // Los 4 canales se asignan automáticamente (no necesitas
  // especificar número de canal en Core 3.x).
  ledcAttach(MOT_L_IN1, PWM_FREQ_HZ, PWM_BITS);
  ledcAttach(MOT_L_IN2, PWM_FREQ_HZ, PWM_BITS);
  ledcAttach(MOT_R_IN1, PWM_FREQ_HZ, PWM_BITS);
  ledcAttach(MOT_R_IN2, PWM_FREQ_HZ, PWM_BITS);

  // Freno activo en ambos motores al arrancar
  motors_stop();

  // ── Encoders ────────────────────────────────────────────────
  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isr_enc_L, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isr_enc_R, CHANGE);

  // ── Botón de armado ─────────────────────────────────────────
  // FALLING = dispara al bajar de HIGH a LOW = momento exacto
  // de la presión. No usar CHANGE (dispararía también al soltar).
  pinMode(BTN_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BTN_PIN), isr_button, FALLING);

  Serial.println("================================================");
  Serial.println("  ESP32-C6 · DRV8833 · Robot MPC");
  Serial.println("  Driver:  IN1=GPIO2  IN2=GPIO3  (motor L)");
  Serial.println("           IN3=GPIO4  IN4=GPIO5  (motor R)");
  Serial.println("           EEP=GPIO8  (enable HIGH)");
  Serial.println("  Estado inicial: IDLE — motores bloqueados");
  Serial.println("  Presiona BTN (GPIO1) para definir origen 0,0");
  Serial.println("================================================");
  delay(100);
}

// ═══════════════════════════════════════════════════════════════
//  LOOP — FSM implementada como switch-case
// ═══════════════════════════════════════════════════════════════
void loop() {
  static uint32_t last_send   = 0;
  static bool     was_running = false;

  // Leer estado volatile una sola vez por ciclo
  robot_state_t state_now = robot_state;

  // Detección de transición — acción única al cambiar de estado
  if (state_now == STATE_RUNNING && !was_running) {
    Serial.println("[FSM] IDLE → RUNNING | origen (0,0) definido");
    was_running = true;
  } else if (state_now == STATE_IDLE && was_running) {
    motors_stop();
    Serial.println("[FSM] RUNNING → IDLE | motores frenados");
    was_running = false;
  }

  switch (state_now) {

    // ─── IDLE ────────────────────────────────────────────────
    // • parse_commands() vacía el buffer UART sin ejecutar nada
    // • Sensores se siguen enviando con ST=0x00
    //   (la RPi los lee pero su MPC queda en pausa)
    case STATE_IDLE:
      parse_commands();
      if (millis() - last_send >= SEND_MS) {
        last_send = millis();
        send_sensor_frame(STATUS_IDLE);
      }
      break;

    // ─── RUNNING ─────────────────────────────────────────────
    // • Comandos PWM de la RPi → motores responden
    // • Frame con ST=0x02 → RPi ejecuta MPC
    // • Encoders fueron zeroed en ISR al entrar aquí
    case STATE_RUNNING:
      parse_commands();
      if (millis() - last_send >= SEND_MS) {
        last_send = millis();
        send_sensor_frame(STATUS_RUNNING);
      }
      break;
  }
}

// ═══════════════════════════════════════════════════════════════
//  TABLA DE CONEXIONES COMPLETA
// ═══════════════════════════════════════════════════════════════
/*
  ESP32-C6         DRV8833         Notas
  GPIO 2     →    IN1             Motor L avance (PWM LEDC)
  GPIO 3     →    IN2             Motor L retroceso (PWM LEDC)
  GPIO 4     →    IN3             Motor R avance (PWM LEDC)
  GPIO 5     →    IN4             Motor R retroceso (PWM LEDC)
  GPIO 8     →    EEP             HIGH = driver activo
  3.3V / 5V  →    VCC             según tu fuente de motor
  GND        →    GND             tierra común

  DRV8833          Motor
  OUT1 / OUT2 →   Motor L  (+ y -)
  OUT3 / OUT4 →   Motor R  (+ y -)

  ESP32-C6         MPU6050
  GPIO 6     →    SDA
  GPIO 7     →    SCL
  3.3V       →    VCC
  GND        →    GND

  ESP32-C6         Encoder L (N20 izq)
  GPIO 10    →    Canal A  (interrupción)
  GPIO 11    →    Canal B  (lectura)
  3.3V       →    VCC
  GND        →    GND

  ESP32-C6         Encoder R (N20 der)
  GPIO 18    →    Canal A  (interrupción)
  GPIO 19    →    Canal B  (lectura)
  3.3V       →    VCC
  GND        →    GND

  ESP32-C6         RPi 4B (UART0)
  GPIO 16 TX →    GPIO 15 RX  (RPi UART0)
  GPIO 17 RX ←    GPIO 14 TX  (RPi UART0)
  GND        ─    GND          (tierra común obligatoria)

  ESP32-C6         Botón
  GPIO 1     →    pulsador NA → GND
*/

// ═══════════════════════════════════════════════════════════════
//  NOTA CORE 2.x (Espressif board package < 3.0.0)
// ═══════════════════════════════════════════════════════════════
/*
  Verifica tu versión: Arduino IDE → Herramientas → Gestor de placas
  → busca "esp32 by Espressif" → mira el número de versión.

  Si es < 3.0.0, en setup() reemplaza los 4 ledcAttach() por:

    ledcSetup(0, PWM_FREQ_HZ, PWM_BITS); ledcAttachPin(MOT_L_IN1, 0);
    ledcSetup(1, PWM_FREQ_HZ, PWM_BITS); ledcAttachPin(MOT_L_IN2, 1);
    ledcSetup(2, PWM_FREQ_HZ, PWM_BITS); ledcAttachPin(MOT_R_IN1, 2);
    ledcSetup(3, PWM_FREQ_HZ, PWM_BITS); ledcAttachPin(MOT_R_IN2, 3);

  Y en drv_set(), reemplaza ledcWrite(pin, duty) por la versión
  de canal explícito:
    // Motor L:  canales 0 y 1
    // Motor R:  canales 2 y 3
  
  Ejemplo para drv_set con Core 2.x:
    void drv_set_2x(uint8_t ch_a, uint8_t ch_b, int16_t val) {
      int32_t duty = ((int32_t)abs(val) * PWM_MAX_DUTY) / 1000;
      if (duty > PWM_MAX_DUTY) duty = PWM_MAX_DUTY;
      if      (val > 0) { ledcWrite(ch_a, duty); ledcWrite(ch_b, 0); }
      else if (val < 0) { ledcWrite(ch_a, 0);    ledcWrite(ch_b, duty); }
      else              { ledcWrite(ch_a, PWM_MAX_DUTY); ledcWrite(ch_b, PWM_MAX_DUTY); }
    }
    // Llamar con: drv_set_2x(0, 1, pwm_L);  drv_set_2x(2, 3, pwm_R);
*/