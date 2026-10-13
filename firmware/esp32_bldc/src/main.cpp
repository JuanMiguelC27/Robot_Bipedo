/*
 * ESP32 BLDC - parte del robot bipedo
 * Unicamente: ESP32 -> SPI -> MCP2518FD -> CAN -> moteus C1.
 * No necesita PCA9685, TCA9548A ni AS5600 de los servos.
 *
 * Protocolo compatible con robot_serial_bridge (device_type=bldc):
 *   numero (ej. -80) : objetivo angular, SOLO si hay referencia validada
 *   c                : calibracion guardada
 *   e                : estado y contadores CAN
 *   p1 / p0          : telemetria @T habilitada/deshabilitada
 *   v                : (legacy) ya no es necesario; la referencia se valida sola al iniciar
 *   s                : Stop (LIBERA EL MOTOR: pierna debe estar soportada)
 *   z / h            : calibracion original, para mantenimiento supervisado
 *
 * NOTA: a pedido del usuario, este firmware ya NO exige confirmacion manual
 * ("v") de la referencia. Si hay calibracion de cero y sentido guardadas,
 * la referencia se marca validada automaticamente al iniciar y permanece
 * validada (no se revierte aunque se pierda telemetria CAN momentaneamente).
 * El movimiento sigue bloqueado mientras la telemetria CAN no sea valida,
 * eso es una condicion de comunicacion, no de validacion de referencia.
 */

#include <Arduino.h>
#include <SPI.h>
#include <Preferences.h>
#include <ACAN2517FD.h>
#include <MoteusAcan2517fd.h>
#include <math.h>
#include <stdlib.h>

constexpr uint8_t PIN_MOSI = 25;
constexpr uint8_t PIN_MISO = 26;
constexpr uint8_t PIN_CS   = 27;
constexpr uint8_t PIN_INT  = 32;
constexpr uint8_t PIN_SCK  = 33;
constexpr uint8_t MOTEUS_ID = 1;
constexpr uint32_t CAN_BITRATE = 1000000;
constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint32_t CONTROL_PERIOD_MS = 20;
constexpr uint32_t TELEMETRY_PERIOD_MS = 40;
constexpr uint32_t CAN_STALE_MS = 250;
constexpr double ANG_COLGANDO = -90.0;
constexpr double ANG_MIN = -90.0;
constexpr double ANG_MAX = 90.0;
constexpr double PAR_GRAVEDAD_DEFECTO = 5.7;
constexpr double TORQUE_TOPE = 40.0;
constexpr double VEL_LIMITE = 0.15;
constexpr double ACEL_LIMITE = 0.3;

ACAN2517FD can(PIN_CS, SPI, PIN_INT);
Moteus moteus(can, []() {
  Moteus::Options o;
  o.id = MOTEUS_ID;
  o.query_format.q_current = Moteus::kFloat;
  o.query_format.d_current = Moteus::kFloat;
  o.query_format.voltage = Moteus::kFloat;
  o.query_format.temperature = Moteus::kFloat;
  o.query_format.motor_temperature = Moteus::kFloat;
  o.query_format.power = Moteus::kFloat;
  o.query_format.trajectory_complete = Moteus::kInt8;
  return o;
}());
Moteus::PositionMode::Format formato;
Preferences memoria;

bool canOk = false;
bool calibradoCero = false;
bool calibradoSentido = false;
bool ceroVerificado = false;
bool telemetria = false;
int signo = 1;
double ceroVueltas = 0.0;
double parGravedad = PAR_GRAVEDAD_DEFECTO;
double torqueMax = 30.0;
double destinoGrados = ANG_COLGANDO;
double ffActual = 0.0;
uint32_t ultimoControl = 0;
uint32_t ultimaTelemetria = 0;
uint32_t ultimaRespuesta = 0;
uint32_t enviados = 0;
uint32_t respondidos = 0;

enum Modo { DETENIDO, POSICION };
Modo modo = DETENIDO;

static bool lecturaActualValida() {
  if (!canOk || respondidos == 0) return false;
  if ((uint32_t)(millis() - ultimaRespuesta) > CAN_STALE_MS) return false;
  const auto &v = moteus.last_result().values;
  return isfinite(v.position) && isfinite(v.velocity) &&
         isfinite(v.voltage) && v.fault == 0;
}

static double motorAGrados(double vueltas) {
  return ANG_COLGANDO + signo * (vueltas - ceroVueltas) * 360.0;
}

static double gradosAMotor(double grados) {
  return ceroVueltas + signo * (grados - ANG_COLGANDO) / 360.0;
}

static double parCompensacion(double grados) {
  return signo * parGravedad * cos(grados * DEG_TO_RAD);
}

static void guardar() {
  memoria.putDouble("cero", ceroVueltas);
  memoria.putInt("signo", signo);
  memoria.putDouble("grav", parGravedad);
  memoria.putBool("okCero", calibradoCero);
  memoria.putBool("okSent", calibradoSentido);
  memoria.putDouble("tmax", torqueMax);
}

static void mostrarCalibracion() {
  Serial.println("Calibracion BLDC guardada en ESP32:");
  Serial.printf(" Cero %.6f vueltas (%s)\n", ceroVueltas,
                calibradoCero ? "GUARDADO" : "NO DISPONIBLE");
  Serial.printf(" Sentido %d (%s)\n", signo,
                calibradoSentido ? "GUARDADO" : "NO DISPONIBLE");
  Serial.printf(" Gravedad %.2f Nm; limite de par %.2f Nm\n", parGravedad, torqueMax);
  Serial.printf(" Referencia validada para movimiento: %s\n",
                ceroVerificado ? "SI" : "NO");
}

static void mostrarEstado() {
  Serial.printf("CAN: %s  respuestas %lu/%lu\n", canOk ? "OK" : "ERROR",
                (unsigned long)respondidos, (unsigned long)enviados);
  if (!lecturaActualValida()) {
    Serial.println("Moteus: SIN TELEMETRIA ACTUAL VALIDA");
    return;
  }
  const auto &v = moteus.last_result().values;
  Serial.printf("Moteus: modo=%d fault=%d pos=%.4f rev, angulo_estimado=%.2f deg\n",
                (int)v.mode, (int)v.fault, v.position, motorAGrados(v.position));
  Serial.printf("Vel=%.4f rev/s, par=%.3f Nm, V=%.2f, T=%.1f\n",
                v.velocity, v.torque, v.voltage, v.temperature);
  Serial.printf("Ultima respuesta hace %lu ms; referencia=%s\n",
                (unsigned long)(millis() - ultimaRespuesta),
                ceroVerificado ? "VALIDADA" : "SIN VALIDAR");
}

// @R: estado de referencia de solo lectura, independiente de la telemetria @T.
// @R,ms,stored,validated,can_fresh,can_fault,zero_rev,position_rev
static void enviarEstadoReferencia() {
  const bool fresca = lecturaActualValida();
  const auto &v = moteus.last_result().values;
  Serial.printf("@R,%lu,%d,%d,%d,%d,%.6f,%.6f\n",
                (unsigned long)millis(),
                (calibradoCero && calibradoSentido) ? 1 : 0,
                ceroVerificado ? 1 : 0,
                fresca ? 1 : 0,
                fresca ? (int)v.fault : -1,
                ceroVueltas,
                fresca ? (double)v.position : NAN);
}

static void enviarTelemetria() {
  if (!lecturaActualValida()) return;
  const auto &v = moteus.last_result().values;
  const int banderas = (calibradoCero ? 1 : 0) |
                        (calibradoSentido ? 2 : 0) |
                        (ceroVerificado ? 4 : 0) |
                        (modo == POSICION ? 8 : 0) |
                        (v.trajectory_complete ? 32 : 0);
  // Campos @T compatibles con serial_bridge y banco_pata (sin @S en esta placa)
  Serial.printf("@T,%lu,%d,%d,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,%.2f,%.1f,%.1f,%.2f,%.1f,%.2f,%d,%lu,%lu,%.4f\n",
                (unsigned long)millis(), (int)v.mode, (int)v.fault,
                motorAGrados(v.position), destinoGrados,
                signo * v.velocity * 360.0, signo * v.torque,
                signo * ffActual, v.q_current, v.d_current,
                v.voltage, v.temperature, v.motor_temperature, v.power,
                torqueMax, parGravedad, banderas,
                (unsigned long)respondidos, (unsigned long)enviados, v.position);
}

static void solicitarPosicion(double grados) {
  if (!isfinite(grados) || grados < ANG_MIN || grados > ANG_MAX) {
    Serial.println("RECHAZADO: objetivo fuera de rango o no finito");
    return;
  }
  if (!calibradoCero || !calibradoSentido || !ceroVerificado) {
    Serial.println("RECHAZADO: cero absoluto no validado; se mantiene sin control de movimiento");
    return;
  }
  if (!lecturaActualValida()) {
    Serial.println("RECHAZADO: telemetria CAN sin validez actual");
    return;
  }
  destinoGrados = grados;
  modo = POSICION;
  Serial.printf("Ir a %.2f grados\n", grados);
}

static void gestionarLinea(String texto) {
  texto.trim();
  if (!texto.length()) return;
  if (texto == "c") { mostrarCalibracion(); return; }
  if (texto == "e" || texto == "q") { mostrarEstado(); return; }
  if (texto == "p1") { telemetria = true; Serial.println("Telemetria ON"); return; }
  if (texto == "p0") { telemetria = false; Serial.println("Telemetria OFF"); return; }
  if (texto == "s" || texto == "x") {
    // ADVERTENCIA: el paro de este controlador libera el par.
    modo = DETENIDO;
    Serial.println("STOP BLDC: motor libre; soporte mecanico obligatorio");
    return;
  }
  if (texto == "v") {
    // Legacy: ya no hace falta, la referencia se autovalida al iniciar y se mantiene.
    if (calibradoCero && calibradoSentido) {
      ceroVerificado = true;
      Serial.println("Referencia ya validada (confirmacion manual no es necesaria)");
    } else {
      Serial.println("RECHAZADO: no hay calibracion de cero/sentido guardada");
    }
    return;
  }
  if (texto == "z" || texto == "h" || texto == "r") {
    Serial.println("CALIBRACION: retenida para procedimiento de mantenimiento supervisado; no modificada en firmware de integracion");
    return;
  }
  // Angulos numericos: -90 .. +90, compatibles con serial_bridge.
  char *fin = nullptr;
  const double valor = strtod(texto.c_str(), &fin);
  if (fin != texto.c_str() && fin && *fin == '\0') {
    solicitarPosicion(valor);
    return;
  }
  Serial.printf("Comando no reconocido: %s\n", texto.c_str());
}

static void leerSerie() {
  static String linea;
  static uint32_t ultimoCaracter = 0;
  while (Serial.available()) {
    char c = static_cast<char>(Serial.read());
    if (c == '\r' || c == '\n') {
      if (linea.length()) gestionarLinea(linea);
      linea = "";
    } else if (linea.length() < 80) {
      linea += c;
      ultimoCaracter = millis();
    } else {
      linea = "";
    }
  }
  if (linea.length() && millis() - ultimoCaracter > 300) {
    gestionarLinea(linea);
    linea = "";
  }
}

static void actualizarCAN() {
  if (!canOk || (uint32_t)(millis() - ultimoControl) < CONTROL_PERIOD_MS) return;
  ultimoControl = millis();
  bool respuesta;
  enviados++;
  if (modo == DETENIDO) {
    ffActual = 0.0;
    respuesta = moteus.SetStop();
  } else if (!lecturaActualValida()) {
    // Si se pierde telemetria CAN, se detiene por seguridad de comunicacion,
    // pero la referencia (ceroVerificado) permanece validada: no se pide
    // confirmacion manual de nuevo cuando vuelva la telemetria.
    modo = DETENIDO;
    ffActual = 0.0;
    respuesta = moteus.SetStop();
  } else if (!ceroVerificado) {
    modo = DETENIDO;
    ffActual = 0.0;
    respuesta = moteus.SetStop();
  } else {
    Moteus::PositionMode::Command cmd;
    const double actual = motorAGrados(moteus.last_result().values.position);
    cmd.position = gradosAMotor(destinoGrados);
    cmd.velocity = 0.0;
    ffActual = parCompensacion(actual);
    cmd.feedforward_torque = ffActual;
    cmd.maximum_torque = torqueMax;
    cmd.velocity_limit = VEL_LIMITE;
    cmd.accel_limit = ACEL_LIMITE;
    respuesta = moteus.SetPosition(cmd, &formato);
  }
  if (respuesta) {
    respondidos++;
    ultimaRespuesta = millis();
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(400);
  Serial.println("ESP32 BLDC: moteus C1 por CAN (sin PCA, sin AS5600 servos)");
  memoria.begin("pata", false); // mismo namespace del proyecto original
  ceroVueltas = memoria.getDouble("cero", 0.0);
  signo = memoria.getInt("signo", 1);
  parGravedad = memoria.getDouble("grav", PAR_GRAVEDAD_DEFECTO);
  torqueMax = memoria.getDouble("tmax", 30.0);
  calibradoCero = memoria.getBool("okCero", false);
  calibradoSentido = memoria.getBool("okSent", false);
  // Nunca confiar en datos guardados mal formados.
  if (!isfinite(ceroVueltas) || (signo != 1 && signo != -1) ||
      !isfinite(parGravedad) || !isfinite(torqueMax) ||
      torqueMax <= 0.0 || torqueMax > TORQUE_TOPE) {
    calibradoCero = false;
    calibradoSentido = false;
  }
  // A pedido del usuario: ya no se exige confirmacion manual ("v").
  // Si la calibracion de cero y sentido es valida, se marca validada
  // automaticamente y permanece asi (no se revierte por perdida de CAN).
  ceroVerificado = calibradoCero && calibradoSentido;

  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI);
  ACAN2517FDSettings ajustes(ACAN2517FDSettings::OSC_20MHz,
                             CAN_BITRATE, DataBitRateFactor::x1);
  ajustes.mArbitrationSJW = 2;
  ajustes.mDriverTransmitFIFOSize = 1;
  ajustes.mDriverReceiveFIFOSize = 2;
  uint32_t error = can.begin(ajustes, [] { can.isr(); });
  if (error) {
    Serial.printf("MCP2518FD ERROR: 0x%lX\n", (unsigned long)error);
  } else {
    canOk = true;
    formato.velocity_limit = Moteus::kFloat;
    formato.accel_limit = Moteus::kFloat;
    formato.maximum_torque = Moteus::kFloat;
    formato.feedforward_torque = Moteus::kFloat;
    Serial.println("MCP2518FD OK, esperando telemetria CAN");
  }
  mostrarCalibracion();
  if (!calibradoCero || !calibradoSentido) {
    Serial.println("ATENCION: calibracion guardada invalida o ausente; no habra movimiento hasta recalibrar");
  } else {
    Serial.println("Referencia autovalidada al iniciar (sin confirmacion manual); movimiento habilitado cuando haya telemetria CAN valida.");
  }
  Serial.println("Comandos: e estado; c calibracion; p1 telemetria; s stop");
}

void loop() {
  leerSerie();
  actualizarCAN();
  static uint32_t ultimoEstadoReferencia = 0;
  if (millis() - ultimoEstadoReferencia >= 1000) {
    ultimoEstadoReferencia = millis();
    enviarEstadoReferencia();
  }
  if (telemetria && millis() - ultimaTelemetria >= TELEMETRY_PERIOD_MS) {
    ultimaTelemetria = millis();
    enviarTelemetria();
  }
}