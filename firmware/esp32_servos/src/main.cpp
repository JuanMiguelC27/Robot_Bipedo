#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

/*
  ============================================================
  ESP32 SERVOS
  - Controla dos articulaciones lógicas:
      a -> Hip Pitch
      b -> Knee Pitch
  - Usa PCA9685 para generar PWM
  - Usa TCA9548A + 2 x AS5600 para leer posición
  - Recibe comandos seriales:
      a<grados>   ej: a10.0
      b<grados>   ej: b-15.0
      e           estado completo
      q           estado corto
      x           soltar servos (PWM off)
      p1          habilitar telemetría @S
      p0          deshabilitar telemetría @S
      ?           ayuda

  - Envía telemetría periódica:
      @S,<ms>,<A_dest>,<A_cmd>,<A_enc>,<A_raw>,<A_flags>,<B_dest>,<B_cmd>,<B_enc>,<B_raw>,<B_flags>,nan,nan,nan

  NOTAS IMPORTANTES:
  1. Ajusta los canales del PCA según tu cableado real.
  2. Ajusta los trims, inversiones y límites según tu mecánica.
  3. Este firmware NO asume nada sobre /dev/ttyUSB0 o /dev/ttyUSB1.
     Eso se resuelve del lado de ROS/PC.
  ============================================================
*/

// ==========================
// Pines / buses
// ==========================
static const int PIN_SDA = 21;
static const int PIN_SCL = 22;

static const uint8_t ADDR_PCA9685 = 0x40;
static const uint8_t ADDR_TCA9548A = 0x70;
static const uint8_t ADDR_AS5600   = 0x36;

// ==========================
// PCA9685
// ==========================
Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(ADDR_PCA9685);

// Frecuencia real calibrada del PCA9685 (la que tú mediste)
static const uint32_t PCA_OSCILLATOR_HZ = 26579680UL;
static const float PCA_SERVO_FREQ = 50.0f;

// Canales PCA9685
// Canales recuperados del firmware legacy (CFG): Arriba 12/13, Abajo 8/9.
// Verificar con el esquematico antes de ensayos fisicos.
static const uint8_t CH_A_1 = 12;   // servo 1 de la articulación A
static const uint8_t CH_A_2 = 13;   // servo 2 de la articulación A
static const uint8_t CH_B_1 = 8;   // servo 1 de la articulación B
static const uint8_t CH_B_2 = 9;   // servo 2 de la articulación B

// ==========================
// Conversión servo
// ==========================
// Ajusta estos pulsos si ya tienes una calibración mejor.
static const uint16_t SERVO_MIN_COUNT = 102;   // aprox 500us
static const uint16_t SERVO_MAX_COUNT = 512;   // aprox 2500us

// Comandos esperados desde ROS/serial_bridge:
//   a0  -> centro
//   a+  -> un lado
//   a-  -> otro lado
// Por eso, internamente convertimos:
//   comando_deg in [-90, 90]  -->  servo_deg in [0, 180]
static const float CMD_MIN_DEG = -90.0f;
static const float CMD_MAX_DEG =  90.0f;

// Ajustes por articulación
static const bool  INV_A = false;    // invertir signo si hace falta
static const bool  INV_B = false;
static const float TRIM_A_DEG = 0.0f;
static const float TRIM_B_DEG = 0.0f;

// Límites lógicos permitidos
static const float LIM_A_MIN = -90.0f;
static const float LIM_A_MAX =  90.0f;
static const float LIM_B_MIN = -90.0f;
static const float LIM_B_MAX =  90.0f;

// ==========================
// TCA9548A + AS5600
// ==========================
static const uint8_t TCA_CH_A = 0;   // encoder articulación A
static const uint8_t TCA_CH_B = 1;   // encoder articulación B

// Si luego quieres alinear a "cero mecánico", ajustas estos offsets.
static const int ENCODER_A_ZERO_RAW = 0;
static const int ENCODER_B_ZERO_RAW = 0;
static const int ENCODER_A_SIGN = +1;
static const int ENCODER_B_SIGN = +1;

// ==========================
// Estado global
// ==========================
bool pca_ok = false;
bool telemetria_activa = true;
bool servos_activos = false;
bool activo_a = false;
bool activo_b = false;
float vel_a_deg_s = 20.0f;
float vel_b_deg_s = 20.0f;

// consignas lógicas (las que llegan por serial)
float target_a_cmd_deg = 0.0f;
float target_b_cmd_deg = 0.0f;

// estado real aplicado (último valor enviado al servo)
float current_a_cmd_deg = 0.0f;
float current_b_cmd_deg = 0.0f;

// suavizado opcional
static const float MAX_SPEED_DEG_S = 180.0f; // tope interno de cambio de consigna
unsigned long last_update_ms = 0;
unsigned long last_telemetry_ms = 0;

// ==========================
// Utilidades
// ==========================
float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

float wrapDeg360(float x) {
  while (x < 0.0f)   x += 360.0f;
  while (x >= 360.0f) x -= 360.0f;
  return x;
}

uint16_t angleDegToPwmCount(float servo_deg) {
  servo_deg = clampf(servo_deg, 0.0f, 180.0f);
  float ratio = servo_deg / 180.0f;
  return (uint16_t)(SERVO_MIN_COUNT + ratio * (SERVO_MAX_COUNT - SERVO_MIN_COUNT));
}

float commandDegToServoDegA(float cmd_deg) {
  cmd_deg = clampf(cmd_deg, LIM_A_MIN, LIM_A_MAX);
  float s = INV_A ? -cmd_deg : cmd_deg;
  return clampf(90.0f + TRIM_A_DEG + s, 0.0f, 180.0f);
}

float commandDegToServoDegB(float cmd_deg) {
  cmd_deg = clampf(cmd_deg, LIM_B_MIN, LIM_B_MAX);
  float s = INV_B ? -cmd_deg : cmd_deg;
  return clampf(90.0f + TRIM_B_DEG + s, 0.0f, 180.0f);
}

// ==========================
// I2C helpers
// ==========================
bool i2cDevicePresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

bool tcaSelect(uint8_t channel) {
  if (channel > 7) return false;
  Wire.beginTransmission(ADDR_TCA9548A);
  Wire.write(1 << channel);
  return (Wire.endTransmission() == 0);
}

bool as5600Read8(uint8_t reg, uint8_t &value) {
  Wire.beginTransmission(ADDR_AS5600);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;

  if (Wire.requestFrom((int)ADDR_AS5600, 1) != 1) return false;
  value = Wire.read();
  return true;
}

bool as5600Read16(uint8_t reg, uint16_t &value) {
  Wire.beginTransmission(ADDR_AS5600);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;

  if (Wire.requestFrom((int)ADDR_AS5600, 2) != 2) return false;
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  value = ((uint16_t)hi << 8) | lo;
  return true;
}

struct EncoderData {
  bool present = false;
  bool magnetDetected = false;
  bool magnetWeak = false;
  bool magnetStrong = false;
  uint16_t raw = 0;
  float aligned_deg = 0.0f;
};

bool readAS5600OnChannel(uint8_t tca_channel,
                         int zero_raw,
                         int sign,
                         EncoderData &out) {
  out = EncoderData();

  if (!tcaSelect(tca_channel)) {
    return false;
  }

  uint8_t status = 0;
  uint16_t raw_angle = 0;

  // STATUS = 0x0B
  // RAW_ANGLE = 0x0C/0x0D
  bool ok_status = as5600Read8(0x0B, status);
  bool ok_raw    = as5600Read16(0x0C, raw_angle);

  if (!(ok_status && ok_raw)) {
    return false;
  }

  out.present = true;
  out.raw = raw_angle & 0x0FFF;

  // Bits status AS5600:
  // MD = bit 5 (0x20) magnet detected
  // ML = bit 4 (0x10) magnet too weak
  // MH = bit 3 (0x08) magnet too strong
  out.magnetDetected = status & 0x20;
  out.magnetWeak     = status & 0x10;
  out.magnetStrong   = status & 0x08;

  int delta = (int)out.raw - zero_raw;
  while (delta < 0) delta += 4096;
  while (delta >= 4096) delta -= 4096;

  float deg = (360.0f * delta) / 4096.0f;
  if (sign < 0) deg = -deg;
  out.aligned_deg = wrapDeg360(deg);

  return true;
}

// ==========================
// PCA / Servos
// ==========================
void servoWriteLogicalA(float cmd_deg) {
  if (!pca_ok || !activo_a) return;
  float servo_deg = commandDegToServoDegA(cmd_deg);
  uint16_t pwm = angleDegToPwmCount(servo_deg);

  pca.setPWM(CH_A_1, 0, pwm);
  pca.setPWM(CH_A_2, 0, pwm);
}

void servoWriteLogicalB(float cmd_deg) {
  if (!pca_ok || !activo_b) return;
  float servo_deg = commandDegToServoDegB(cmd_deg);
  uint16_t pwm = angleDegToPwmCount(servo_deg);

  pca.setPWM(CH_B_1, 0, pwm);
  pca.setPWM(CH_B_2, 0, pwm);
}

void servoReleaseAll() {
  if (!pca_ok) return;
  pca.setPWM(CH_A_1, 0, 0);
  pca.setPWM(CH_A_2, 0, 0);
  pca.setPWM(CH_B_1, 0, 0);
  pca.setPWM(CH_B_2, 0, 0);
}

// ==========================
// Estado / Telemetría
// ==========================
String encoderStatusString(const EncoderData &enc) {
  if (!enc.present) return "no responde";
  if (!enc.magnetDetected) return "sin iman";
  if (enc.magnetWeak)   return "iman debil";
  if (enc.magnetStrong) return "iman fuerte";
  return "campo OK, sin alinear";
}

void printShortState() {
  EncoderData a, b;
  readAS5600OnChannel(TCA_CH_A, ENCODER_A_ZERO_RAW, ENCODER_A_SIGN, a);
  readAS5600OnChannel(TCA_CH_B, ENCODER_B_ZERO_RAW, ENCODER_B_SIGN, b);

  Serial.println("Estado servos:");
  Serial.printf("  PCA9685: %s\n", pca_ok ? "OK" : "NO detectado");

  if (a.present) {
    Serial.printf("  Arriba: target %.2f deg | enc raw %u | enc %.2f deg | %s\n",
                  current_a_cmd_deg, a.raw, a.aligned_deg, encoderStatusString(a).c_str());
  } else {
    Serial.printf("  Arriba: target %.2f deg | encoder no responde (canal %u)\n",
                  current_a_cmd_deg, TCA_CH_A);
  }

  if (b.present) {
    Serial.printf("  Abajo:  target %.2f deg | enc raw %u | enc %.2f deg | %s\n",
                  current_b_cmd_deg, b.raw, b.aligned_deg, encoderStatusString(b).c_str());
  } else {
    Serial.printf("  Abajo:  target %.2f deg | encoder no responde (canal %u)\n",
                  current_b_cmd_deg, TCA_CH_B);
  }
}

void printHelp() {
  Serial.println();
  Serial.println("--- Comandos ESP32 SERVOS --------------------");
  Serial.println("  a<deg>    -> consigna articulacion A (ej: a10, a-20)");
  Serial.println("  b<deg>    -> consigna articulacion B (ej: b15, b-10)");
  Serial.println("  ax/bx     -> soltar solo el par seleccionado");
  Serial.println("  av20/bv20 -> velocidad maxima de consigna de cada par");
  Serial.println("  x         -> soltar todos los servos (PWM off)");
  Serial.println("  e         -> estado completo");
  Serial.println("  q         -> estado corto");
  Serial.println("  p1        -> telemetria @S ON");
  Serial.println("  p0        -> telemetria @S OFF");
  Serial.println("  ?         -> ayuda");
  Serial.println("---------------------------------------------");
  Serial.println();
}

void printBootInfo() {
  Serial.println("ESP32 SERVOS: PCA9685 + TCA9548A + 2x AS5600");
  Serial.printf("I2C SDA=%d SCL=%d\n", PIN_SDA, PIN_SCL);
  Serial.printf("PCA9685 addr=0x%02X | TCA9548A addr=0x%02X\n", ADDR_PCA9685, ADDR_TCA9548A);
  Serial.printf("Canales A=(%u,%u), B=(%u,%u)\n", CH_A_1, CH_A_2, CH_B_1, CH_B_2);
  Serial.printf("Telemetria @S: %s\n", telemetria_activa ? "ON" : "OFF");
}

void sendTelemetry() {
  if (!telemetria_activa) return;

  EncoderData a, b;
  const bool oka = readAS5600OnChannel(
      TCA_CH_A, ENCODER_A_ZERO_RAW, ENCODER_A_SIGN, a);
  const bool okb = readAS5600OnChannel(
      TCA_CH_B, ENCODER_B_ZERO_RAW, ENCODER_B_SIGN, b);

  // Banderas compatibles con banco_pata.py:
  // bit 0: PWM habilitado, bit 1: lectura I2C valida,
  // bit 2: iman detectado, bit 3: campo debil,
  // bit 4: campo fuerte, bit 5: referencia angular alineada,
  // bit 6: el comando aun se esta interpolando.
  // Esta version NO guarda ceros de encoder en Preferences:
  // por eso el bit 5 queda en 0 y la GUI muestra 'sin alinear'.
  auto flags = [&](const EncoderData &e, bool ok, float target, float current, bool activo) -> unsigned {
    unsigned f = activo && pca_ok ? 1u : 0u;
    if (ok && e.present) {
      f |= 2u;
      if (e.magnetDetected) f |= 4u;
      if (e.magnetWeak) f |= 8u;
      if (e.magnetStrong) f |= 16u;
    }
    if (fabsf(target - current) > 0.1f && activo) f |= 64u;
    return f;
  };

  const unsigned fa = flags(a, oka, target_a_cmd_deg, current_a_cmd_deg, activo_a);
  const unsigned fb = flags(b, okb, target_b_cmd_deg, current_b_cmd_deg, activo_b);

  // El formato es:
  // @S,ms,a_dest,a_cmd,a_enc,a_raw,a_flags,
  //     b_dest,b_cmd,b_enc,b_raw,b_flags,i1,i2,i3
  // Sin calibracion de encoder, a_enc y b_enc son la lectura
  // relativa al raw 0, NO un angulo articular mecanico verificado.
  // Las corrientes no se miden en este firmware: NaN.
  Serial.printf(
    "@S,%lu,%.3f,%.3f,%.3f,%u,%u,%.3f,%.3f,%.3f,%u,%u,nan,nan,nan\n",
    (unsigned long)millis(),
    target_a_cmd_deg, current_a_cmd_deg,
    (oka && a.present) ? a.aligned_deg : NAN,
    (unsigned)(oka && a.present ? a.raw : 0), fa,
    target_b_cmd_deg, current_b_cmd_deg,
    (okb && b.present) ? b.aligned_deg : NAN,
    (unsigned)(okb && b.present ? b.raw : 0), fb
  );
}

// ==========================
// Parser serial
// ==========================
void handleCommand(const String &lineRaw) {
  String line = lineRaw;
  line.trim();
  if (line.length() == 0) return;

  if (line == "?") {
    printHelp();
    return;
  }

  if (line == "e" || line == "q") {
    printShortState();
    return;
  }

  if (line == "x") {
    servos_activos = false;
    activo_a = activo_b = false;
    servoReleaseAll();
    Serial.println("Servos liberados (PWM OFF).");
    return;
  }

  if (line == "p1") {
    telemetria_activa = true;
    Serial.println("Telemetria @S activada.");
    return;
  }

  if (line == "p0") {
    telemetria_activa = false;
    Serial.println("Telemetria @S desactivada.");
    return;
  }

  const char cmd = line.charAt(0);
  if (cmd == 'a' || cmd == 'b') {
    const bool esA = cmd == 'a';
    const String argumento = line.substring(1);
    if (argumento == "x") {
      if (esA) {
        activo_a = false;
        if (pca_ok) { pca.setPWM(CH_A_1, 0, 0); pca.setPWM(CH_A_2, 0, 0); }
      } else {
        activo_b = false;
        if (pca_ok) { pca.setPWM(CH_B_1, 0, 0); pca.setPWM(CH_B_2, 0, 0); }
      }
      servos_activos = activo_a || activo_b;
      Serial.printf("[ACK] Par %c liberado\n", cmd);
      return;
    }
    if (argumento.startsWith("v")) {
      const String valor = argumento.substring(1);
      char *fin = nullptr;
      float vel = strtof(valor.c_str(), &fin);
      if (valor.isEmpty() || !fin || *fin != '\0' || !isfinite(vel) || vel <= 0 || vel > MAX_SPEED_DEG_S) {
        Serial.println("[RECHAZADO] Velocidad invalida");
        return;
      }
      if (esA) vel_a_deg_s = vel; else vel_b_deg_s = vel;
      Serial.printf("[ACK] Velocidad %c = %.2f deg/s\n", cmd, vel);
      return;
    }
    // No aceptar 'az', 'bz' hasta implementar alineacion persistente segura.
    if (argumento == "z") {
      Serial.println("[RECHAZADO] Alineacion AS5600 no implementada");
      return;
    }
    char *fin = nullptr;
    const float deg = strtof(argumento.c_str(), &fin);
    if (argumento.isEmpty() || !fin || *fin != '\0' || !isfinite(deg)) {
      Serial.println("[RECHAZADO] Angulo no numerico");
      return;
    }
    const float minimo = esA ? LIM_A_MIN : LIM_B_MIN;
    const float maximo = esA ? LIM_A_MAX : LIM_B_MAX;
    if (deg < minimo || deg > maximo || !pca_ok) {
      Serial.println("[RECHAZADO] Servo fuera de limites o PCA no disponible");
      return;
    }
    if (esA) {
      target_a_cmd_deg = deg;
      // Retomar desde la ultima consigna, no forzar a cero.
      activo_a = true;
    } else {
      target_b_cmd_deg = deg;
      activo_b = true;
    }
    servos_activos = true;
    Serial.printf("[ACK] %c target = %.3f deg\n", cmd, deg);
    return;
  }

  Serial.printf("[CMD] Comando desconocido '%s'. Escribe ? para ayuda\n", line.c_str());
}

// ==========================
// Setup
// ==========================
void setup() {
  Serial.begin(115200);
  delay(1200);

  // Diagnóstico: bus a 100 kHz para disminuir errores repetidos de requestFrom.
  // No cambia PWM, objetivos ni calibración del AS5600.
  Wire.begin(PIN_SDA, PIN_SCL, 100000);
  delay(100);

  printBootInfo();

  pca_ok = i2cDevicePresent(ADDR_PCA9685);
  if (pca_ok) {
    pca.begin();
    pca.setOscillatorFrequency(PCA_OSCILLATOR_HZ);
    pca.setPWMFreq(PCA_SERVO_FREQ);
    delay(10);
    Serial.println("PCA9685: OK");
  } else {
    Serial.println("PCA9685: NO detectado");
  }

  bool tca_ok = i2cDevicePresent(ADDR_TCA9548A);
  Serial.printf("TCA9548A: %s\n", tca_ok ? "OK" : "NO detectado");

  EncoderData a, b;
  bool oka = readAS5600OnChannel(TCA_CH_A, ENCODER_A_ZERO_RAW, ENCODER_A_SIGN, a);
  bool okb = readAS5600OnChannel(TCA_CH_B, ENCODER_B_ZERO_RAW, ENCODER_B_SIGN, b);

  if (oka && a.present) {
    Serial.printf("Encoder Arriba (canal %u): raw %u | %.2f deg | %s\n",
                  TCA_CH_A, a.raw, a.aligned_deg, encoderStatusString(a).c_str());
  } else {
    Serial.printf("Encoder Arriba (canal %u): no responde\n", TCA_CH_A);
  }

  if (okb && b.present) {
    Serial.printf("Encoder Abajo  (canal %u): raw %u | %.2f deg | %s\n",
                  TCA_CH_B, b.raw, b.aligned_deg, encoderStatusString(b).c_str());
  } else {
    Serial.printf("Encoder Abajo  (canal %u): no responde\n", TCA_CH_B);
  }

  // Home lógico
  current_a_cmd_deg = target_a_cmd_deg = 0.0f;
  current_b_cmd_deg = target_b_cmd_deg = 0.0f;

  // No aplicar PWM en el arranque: esperar consignas validas.
  servoReleaseAll();

  last_update_ms = millis();
  last_telemetry_ms = millis();

  printHelp();
}

// ==========================
// Loop
// ==========================
void loop() {
  // --- parser serial por línea ---
  static String rx;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (rx.length() > 0) {
        handleCommand(rx);
        rx = "";
      }
    } else {
      rx += c;
      if (rx.length() > 100) rx.remove(0); // seguridad simple
    }
  }

  // --- suavizado de consignas ---
  unsigned long now = millis();
  float dt = (now - last_update_ms) * 0.001f;
  if (dt < 0.001f) dt = 0.001f;
  last_update_ms = now;

  const float max_step_a = min(MAX_SPEED_DEG_S, vel_a_deg_s) * dt;
  const float max_step_b = min(MAX_SPEED_DEG_S, vel_b_deg_s) * dt;

  float errA = target_a_cmd_deg - current_a_cmd_deg;
  float errB = target_b_cmd_deg - current_b_cmd_deg;

  if (fabs(errA) > max_step_a) current_a_cmd_deg += (errA > 0 ? max_step_a : -max_step_a);
  else current_a_cmd_deg = target_a_cmd_deg;

  if (fabs(errB) > max_step_b) current_b_cmd_deg += (errB > 0 ? max_step_b : -max_step_b);
  else current_b_cmd_deg = target_b_cmd_deg;

  if (pca_ok) {
    if (activo_a) servoWriteLogicalA(current_a_cmd_deg);
    if (activo_b) servoWriteLogicalB(current_b_cmd_deg);
  }

  // --- telemetría cada 50 ms ---
  if (now - last_telemetry_ms >= 50) {
    last_telemetry_ms = now;
    sendTelemetry();
  }
}