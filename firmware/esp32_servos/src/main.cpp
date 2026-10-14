#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Preferences.h>

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

// Servos reales: RDS51150 (DSSERVO), configurados en modo
// 500-2500us -> 270 grados de giro electrico real (no 180).
// SERVO_MIN_COUNT = 0 grados reales, SERVO_MAX_COUNT = 270 grados reales.
static const float SERVO_FULL_RANGE_DEG = 270.0f;

// Centro mecanico fisico real de cada servo (posicion vertical / home),
// medido en el robot. Con rango electrico 270, el centro geometrico
// queda en 135.
static const float SERVO_CENTER_DEG_A = 135.0f;
static const float SERVO_CENTER_DEG_B = 135.0f;

// Comandos esperados desde ROS/serial_bridge:
//   a0 / b0  -> centro (posicion vertical, 135 grados reales del servo)
//   comando_deg in [-90, 90] grados respecto al centro
// Limita el recorrido real a 45-225 grados (deja margen mecanico
// respecto a los topes electricos 0 y 270 del servo).
static const float CMD_MIN_DEG = -90.0f;
static const float CMD_MAX_DEG =  90.0f;

// Ajustes por articulación
static const bool  INV_A = false;    // invertir signo si hace falta
static const bool  INV_B = false;
static const float TRIM_A_DEG = 0.0f;
static const float TRIM_B_DEG = 0.0f;

// Límites lógicos permitidos (grados respecto al centro, NO grados
// reales del servo)
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
// Referencias capturadas en la posicion HOME mecanica (0 grados ROS).
// No se habilitan por defecto, deben guardarse mediante az / bz.
int encoder_a_zero_raw = 0;
int encoder_b_zero_raw = 0;
bool encoder_a_aligned = false;
bool encoder_b_aligned = false;
Preferences encoder_store;
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

uint16_t angleDegToPwmCount(float servo_deg_real) {
  servo_deg_real = clampf(servo_deg_real, 0.0f, SERVO_FULL_RANGE_DEG);
  float ratio = servo_deg_real / SERVO_FULL_RANGE_DEG;
  return (uint16_t)(SERVO_MIN_COUNT + ratio * (SERVO_MAX_COUNT - SERVO_MIN_COUNT));
}

float commandDegToServoDegA(float cmd_deg) {
  cmd_deg = clampf(cmd_deg, LIM_A_MIN, LIM_A_MAX);
  float s = INV_A ? -cmd_deg : cmd_deg;
  return clampf(SERVO_CENTER_DEG_A + TRIM_A_DEG + s, 0.0f, SERVO_FULL_RANGE_DEG);
}

float commandDegToServoDegB(float cmd_deg) {
  cmd_deg = clampf(cmd_deg, LIM_B_MIN, LIM_B_MAX);
  float s = INV_B ? -cmd_deg : cmd_deg;
  return clampf(SERVO_CENTER_DEG_B + TRIM_B_DEG + s, 0.0f, SERVO_FULL_RANGE_DEG);
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
  // Convencion articular firmada alrededor del cero mecanico.
  // AS5600 es de una sola vuelta; no determina vueltas completas.
  deg = wrapDeg360(deg);
  if (deg > 180.0f) deg -= 360.0f;
  out.aligned_deg = deg;

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
String encoderStatusString(const EncoderData &enc, bool aligned) {
  if (!enc.present) return "no responde";
  if (!enc.magnetDetected) return "sin iman";
  if (enc.magnetWeak)   return "iman debil";
  if (enc.magnetStrong) return "iman fuerte";
  return aligned ? "campo OK, alineado" : "campo OK, sin alinear";
}

void printShortState() {
  EncoderData a, b;
  readAS5600OnChannel(TCA_CH_A, encoder_a_zero_raw, ENCODER_A_SIGN, a);
  readAS5600OnChannel(TCA_CH_B, encoder_b_zero_raw, ENCODER_B_SIGN, b);

  Serial.println("Estado servos:");
  Serial.printf("  Alineacion A=%s (raw0=%d) B=%s (raw0=%d)\n",
      encoder_a_aligned ? "SI" : "NO", encoder_a_zero_raw,
      encoder_b_aligned ? "SI" : "NO", encoder_b_zero_raw);
  Serial.printf("  PCA9685: %s\n", pca_ok ? "OK" : "NO detectado");

  if (a.present) {
    Serial.printf("  Arriba: target %.2f deg | enc raw %u | enc %.2f deg | %s\n",
                  current_a_cmd_deg, a.raw, a.aligned_deg, encoderStatusString(a, encoder_a_aligned).c_str());
  } else {
    Serial.printf("  Arriba: target %.2f deg | encoder no responde (canal %u)\n",
                  current_a_cmd_deg, TCA_CH_A);
  }

  if (b.present) {
    Serial.printf("  Abajo:  target %.2f deg | enc raw %u | enc %.2f deg | %s\n",
                  current_b_cmd_deg, b.raw, b.aligned_deg, encoderStatusString(b, encoder_b_aligned).c_str());
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
  Serial.println("  az / bz   -> registrar HOME (0 deg) del encoder A/B sin mover servos");
  Serial.println("  az / bz   -> alinear encoder en HOME; muestra diagnostico detallado");
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
      TCA_CH_A, encoder_a_zero_raw, ENCODER_A_SIGN, a);
  const bool okb = readAS5600OnChannel(
      TCA_CH_B, encoder_b_zero_raw, ENCODER_B_SIGN, b);

  // Banderas compatibles con banco_pata.py:
  // bit 0: PWM habilitado, bit 1: lectura I2C valida,
  // bit 2: iman detectado, bit 3: campo debil,
  // bit 4: campo fuerte, bit 5: referencia angular alineada,
  // bit 6: el comando aun se esta interpolando.
  // El bit 5 solo se emite con referencia persistida y sensor saludable.
  auto flags = [&](const EncoderData &e, bool ok, float target, float current,
                   bool activo, bool aligned) -> unsigned {
    unsigned f = activo && pca_ok ? 1u : 0u;
    if (ok && e.present) {
      f |= 2u;
      if (e.magnetDetected) f |= 4u;
      if (e.magnetWeak) f |= 8u;
      if (e.magnetStrong) f |= 16u;
      if (aligned && e.magnetDetected && !e.magnetWeak && !e.magnetStrong)
        f |= 32u;
    }
    if (fabsf(target - current) > 0.1f && activo) f |= 64u;
    return f;
  };

  const unsigned fa = flags(a, oka, target_a_cmd_deg, current_a_cmd_deg,
                            activo_a, encoder_a_aligned);
  const unsigned fb = flags(b, okb, target_b_cmd_deg, current_b_cmd_deg,
                            activo_b, encoder_b_aligned);

  // El formato es:
  // @S,ms,a_dest,a_cmd,a_enc,a_raw,a_flags,
  //     b_dest,b_cmd,b_enc,b_raw,b_flags,i1,i2,i3
  // Sin calibracion persistida, el angulo se transmite como NaN;
  // las lecturas RAW y banderas permanecen disponibles para diagnostico.
  // Las corrientes no se miden en este firmware: NaN.
  Serial.printf(
    "@S,%lu,%.3f,%.3f,%.3f,%u,%u,%.3f,%.3f,%.3f,%u,%u,nan,nan,nan\n",
    (unsigned long)millis(),
    target_a_cmd_deg, current_a_cmd_deg,
    (fa & 32u) ? a.aligned_deg : NAN,
    (unsigned)(oka && a.present ? a.raw : 0), fa,
    target_b_cmd_deg, current_b_cmd_deg,
    (fb & 32u) ? b.aligned_deg : NAN,
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
    if (argumento == "z") {
      // Solo registrar el HOME mecanico verificado externamente.
      // No mueve motores ni altera PWM.
      constexpr int N = 6;
      constexpr int TOL_RAW = 8;
      constexpr int MIN_CONSISTENTES = 5;
      const uint8_t channel = esA ? TCA_CH_A : TCA_CH_B;
      const int sign = esA ? ENCODER_A_SIGN : ENCODER_B_SIGN;
      int raw[N];
      EncoderData sample;
      const char *motivo = nullptr;
      int recibidas = 0;
      Serial.printf("[DIAG] Encoder %c: 6 muestras, mediana circular, minimo %d/%d a +/- %d RAW, TCA=%u\n",
                    cmd, MIN_CONSISTENTES, N, TOL_RAW, channel);

      for (int k = 0; k < N; ++k) {
        const bool lectura_ok = readAS5600OnChannel(channel, 0, sign, sample);
        if (!lectura_ok || !sample.present) {
          motivo = "lectura I2C fallida (TCA/STATUS/RAW_ANGLE)";
          Serial.printf("[DIAG] %c muestra %d/%d: ERROR I2C\n", cmd, k + 1, N);
          break;
        }
        if (!sample.magnetDetected || sample.magnetWeak || sample.magnetStrong) {
          motivo = !sample.magnetDetected ? "iman ausente" :
                   sample.magnetWeak ? "campo magnetico debil" : "campo magnetico fuerte";
          Serial.printf("[DIAG] %c muestra %d/%d: RAW=%u, iman=%s, debil=%s, fuerte=%s\n",
                        cmd, k + 1, N, sample.raw,
                        sample.magnetDetected ? "SI" : "NO",
                        sample.magnetWeak ? "SI" : "NO",
                        sample.magnetStrong ? "SI" : "NO");
          break;
        }
        raw[k] = (int)sample.raw;
        ++recibidas;
        Serial.printf("[DIAG] %c muestra %d/%d: RAW=%d, iman=SI, campo=OK\n",
                      cmd, k + 1, N, raw[k]);
        if (k < N - 1) delay(5);
      }
      if (motivo != nullptr || recibidas != N) {
        Serial.printf("[RECHAZADO] %c: %s. Cero anterior conservado.\n",
                      cmd, motivo ? motivo : "faltan muestras");
        return;
      }

      // Los RAW son circulares (4095 -> 0). Buscar una referencia
      // candidata con la mayor cantidad de vecinos y, en ese entorno,
      // calcular la mediana de las 6 lecturas desenrolladas.
      int mejor_cantidad = -1;
      int mejor_referencia = raw[0];
      for (int j = 0; j < N; ++j) {
        int cantidad = 0;
        for (int i = 0; i < N; ++i) {
          int d = abs(raw[i] - raw[j]);
          if (d > 2048) d = 4096 - d;
          if (d <= TOL_RAW) ++cantidad;
        }
        if (cantidad > mejor_cantidad) {
          mejor_cantidad = cantidad;
          mejor_referencia = raw[j];
        }
      }
      // Si hay suficientes vecinos de una candidata, el conjunto es
      // coherente incluso si una lectura aislada fue anomala.
      if (mejor_cantidad < MIN_CONSISTENTES) {
        Serial.printf("[RECHAZADO] %c: muestras dispersas; mayor grupo=%d/%d dentro de %d RAW. Cero anterior conservado.\n",
                      cmd, mejor_cantidad, N, TOL_RAW);
        return;
      }

      int ordenados[N];
      for (int i = 0; i < N; ++i) {
        int d = raw[i] - mejor_referencia;
        if (d > 2048) d -= 4096;
        if (d < -2048) d += 4096;
        ordenados[i] = mejor_referencia + d;
      }
      for (int i = 1; i < N; ++i) {
        int x = ordenados[i];
        int j = i - 1;
        while (j >= 0 && ordenados[j] > x) {
          ordenados[j + 1] = ordenados[j];
          --j;
        }
        ordenados[j + 1] = x;
      }
      const float mediana = 0.5f * (ordenados[2] + ordenados[3]);
      const int candidato = ((int)lroundf(mediana) % 4096 + 4096) % 4096;
      int consistentes = 0;
      int max_desviacion = 0;
      for (int i = 0; i < N; ++i) {
        int d = abs(raw[i] - candidato);
        if (d > 2048) d = 4096 - d;
        if (d <= TOL_RAW) ++consistentes;
        if (d > max_desviacion) max_desviacion = d;
      }
      Serial.printf("[DIAG] %c: mediana=%.2f, RAW0 candidato=%d, consistentes=%d/%d, max desviacion=%d RAW\n",
                    cmd, mediana, candidato, consistentes, N, max_desviacion);
      if (consistentes < MIN_CONSISTENTES) {
        Serial.printf("[RECHAZADO] %c: dispersion excesiva respecto a la mediana. Cero anterior conservado.\n", cmd);
        return;
      }
      // Guardar por separado. Nunca cambiar la referencia RAM si falla NVS.
      const char *key = esA ? "a_zero" : "b_zero";
      if (encoder_store.putUShort(key, (uint16_t)candidato) != sizeof(uint16_t)) {
        Serial.printf("[RECHAZADO] %c: error escribiendo Preferences\n", cmd);
        return;
      }
      if (esA) { encoder_a_zero_raw = candidato; encoder_a_aligned = true; }
      else     { encoder_b_zero_raw = candidato; encoder_b_aligned = true; }
      Serial.printf("[ACK] Encoder %c alineado a HOME 0 deg; RAW0=%d (mediana, %d/%d consistentes, guardado)\n",
                    cmd, candidato, consistentes, N);
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

  // Namespace separado de la calibracion del BLDC.
  encoder_store.begin("enc_servos", false);
  encoder_a_aligned = encoder_store.isKey("a_zero");
  encoder_b_aligned = encoder_store.isKey("b_zero");
  if (encoder_a_aligned) encoder_a_zero_raw = encoder_store.getUShort("a_zero", 0);
  if (encoder_b_aligned) encoder_b_zero_raw = encoder_store.getUShort("b_zero", 0);
  if (encoder_a_zero_raw > 4095) encoder_a_aligned = false;
  if (encoder_b_zero_raw > 4095) encoder_b_aligned = false;

  printBootInfo();
  Serial.printf("AS5600: alineado A=%s, B=%s\n",
                encoder_a_aligned ? "SI" : "NO", encoder_b_aligned ? "SI" : "NO");

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
  bool oka = readAS5600OnChannel(TCA_CH_A, encoder_a_zero_raw, ENCODER_A_SIGN, a);
  bool okb = readAS5600OnChannel(TCA_CH_B, encoder_b_zero_raw, ENCODER_B_SIGN, b);

  if (oka && a.present) {
    Serial.printf("Encoder Arriba (canal %u): raw %u | %.2f deg | %s\n",
                  TCA_CH_A, a.raw, a.aligned_deg, encoderStatusString(a, encoder_a_aligned).c_str());
  } else {
    Serial.printf("Encoder Arriba (canal %u): no responde\n", TCA_CH_A);
  }

  if (okb && b.present) {
    Serial.printf("Encoder Abajo  (canal %u): raw %u | %.2f deg | %s\n",
                  TCA_CH_B, b.raw, b.aligned_deg, encoderStatusString(b, encoder_b_aligned).c_str());
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