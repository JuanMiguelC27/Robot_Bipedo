/*
 * Pata robotica: brushless (moteus-c1 + cicloidal 1:11) + brazo de servos
 * ESP32 + MCP2518FD (CAN) + PCA9685 (servos) + TCA9548A con dos AS5600 (encoders)
 * --------------------------------------------------------------------------------
 * Las tres partes trabajan al mismo tiempo y son independientes:
 *   - Un comando del brushless solo mueve el brushless.
 *   - Un comando del par de ARRIBA solo mueve los dos servos de arriba (juntos).
 *   - Un comando del par de ABAJO solo mueve los dos servos de abajo (juntos).
 *   - Si una parte falla al iniciar (CAN o I2C), las otras siguen funcionando.
 *
 * Conexiones (segun el esquematico):
 *   SPI del MCP2518FD: GPIO 25 SDI, 26 SDO, 27 nCS, 32 INT, 33 SCK
 *   I2C: GPIO 21 SDA, GPIO 22 SCL -> PCA9685 (0x40), TCA9548A (0x70, A0-A2 a GND), BNO085
 *   TCA9548A: canal 0 -> AS5600 (SCL0/SDA0), canal 1 -> AS5600_1 (SCL1/SDA1)
 *   Sensores de corriente: CURRENT1 GPIO 34, CURRENT2 GPIO 35, CURRENT3 GPIO 36 (VP)
 *   (divisor 10k/20k: el ESP32 ve 2/3 del voltaje del sensor)
 *
 * Calibracion del moteus hecha con el mjcanfd-usb-1x y tview (queda guardada en el moteus):
 *   kp 200, kd 4, ki 50, ilimit 12, max_current_A 6, ratio 0.0909091 (1:11), output.source 1
 *
 * ---------------- COMANDOS (escribe y presiona Enter) ----------------
 * BRUSHLESS (pata: 0 = HORIZONTAL, 90 = VERTICAL ARRIBA, -90 = colgando abajo):
 *   numero -90 a 90   ir a ese angulo (ej: 0, 45, 90, -90)
 *   s  stop brushless (la pata cae, sostenla)     v  confirmar cero
 *   z  guardar cero con la pata COLGANDO (queda como -90)   h  sentido con la pata HORIZONTAL (0)
 *   m  medir gravedad en 0 (horizontal)    t<valor> limite de par (ej: t30)
 *   r  borrar calibracion del brushless           c  ver calibracion
 * SERVOS (cada par es independiente):
 *   a<grados>  par de ARRIBA a ese angulo, 0 = centro, -90 a 90 (ej: a45, a-30, a0)
 *   b<grados>  par de ABAJO  a ese angulo, 0 = centro, -90 a 90 (ej: b-50)
 *   ax / bx    soltar ese par (sin fuerza)
 *   az / bz    alinear su encoder con la posicion actual del servo (quieto y activo)
 *   av<v> / bv<v>  velocidad del par en grados por segundo (ej: av40)
 * GENERALES:
 *   e  estado de todo    x  PARAR TODO (brushless libre y servos sueltos)
 *   i  poner en cero los sensores de corriente (sin carga)   g  graficas Teleplot
 *   p1 / p0  telemetria para la interfaz (banco_pata.py)
 *
 * Telemetria (25 Hz):
 *   @T,ms,modo,fault,angulo,destino,vel_gps,par,par_ff,iq,id,voltaje,temp_placa,
 *      temp_motor,potencia,par_max,par_gravedad,banderas,can_ok,can_env,motor_vueltas
 *   @S,ms, [por cada par, arriba y luego abajo: destino,comando,encoder,raw,banderas],
 *      corriente1,corriente2,corriente3
 *   banderas servos: bit0 activo, bit1 encoder responde, bit2 iman detectado, bit3 iman debil,
 *                    bit4 iman fuerte, bit5 encoder alineado, bit6 moviendose
 */

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <math.h>
#include <Preferences.h>
#include <ACAN2517FD.h>
#include <MoteusAcan2517fd.h>

// =====================================================================
// CONFIGURACION: revisa esta parte antes de cargar
// =====================================================================
#define CRISTAL_40MHZ 0        // 0 = cristal de 20 MHz (el de este modulo)

// Pines SPI del MCP2518FD
const uint8_t PIN_MOSI = 25, PIN_MISO = 26, PIN_CS = 27, PIN_INT = 32, PIN_SCK = 33;
const uint8_t MOTEUS_ID = 1;

// Pines I2C y direcciones
const uint8_t PIN_SDA = 21, PIN_SCL = 22;
const uint8_t DIR_PCA9685 = 0x40, DIR_TCA9548A = 0x70, DIR_AS5600 = 0x36;

// Brushless (mismos limites usados en tview: d pos ... v0.15 a0.3)
const double VEL_LIMITE = 0.15, ACEL_LIMITE = 0.3;
const double TORQUE_TOPE = 40.0, PAR_GRAVEDAD_DEFECTO = 5.7;
const double ANG_MIN = -90.0, ANG_MAX = 90.0, TOL_CERO = 2.0;
// Angulos de la pata: 0 = horizontal, 90 = vertical arriba, -90 = colgando.
// El cero fisico se toma con la pata colgando (z), que es una posicion facil de repetir,
// y corresponde a -90 grados.
const double ANG_COLGANDO = -90.0;

// Servos: pulso para 0 y 180 grados (ajusta si tus servos usan otro rango)
const float SERVO_MIN_US = 500.0f, SERVO_MAX_US = 2500.0f;
const float ACEL_SERVOS = 40.0f;   // grados/s^2: menor = arranque y frenado mas suaves

struct ConfigPar {
  const char *nombre;
  char letra;
  uint8_t canal1, canal2;   // salidas del PCA9685 de los dos servos del par (los dos reciben el mismo angulo)
  int trim1_us, trim2_us;   // ajuste fino en microsegundos para alinear los dos servos
  float minimo, maximo;     // limites del par en grados, 0 = centro (pon los de tu mecanismo)
  float velocidad;          // grados por segundo
  uint8_t canalEncoder;     // canal del TCA9548A donde esta su AS5600
  int signoEncoder;         // +1 o -1 para que el encoder crezca igual que el servo
};
ConfigPar CFG[2] = {
  // nombre    letra canales trims  min   max  vel  enc signo
  {"Arriba",   'a', 12, 13,  0, 0,  -90,  90, 20,  0,  1},
  {"Abajo",    'b',  8, 9,   0, 0,  -90,  90, 20,  1,  1},
};

// Sensores de corriente (ACS712 de 20 A: 0.100 V/A; de 5 A: 0.185; de 30 A: 0.066)
const uint8_t PIN_CORRIENTE[3] = {34, 35, 36};
const float DIVISOR_CORRIENTE = 1.5f;     // 10k/20k
const float SENS_V_POR_A = 0.100f;

const uint32_t PERIODO_MS = 20, PERIODO_SERVOS_MS = 20, PERIODO_GRAFICA_MS = 40, PERIODO_TELEMETRIA_MS = 40;
const double TOL_LLEGADA = 1.0;
const uint32_t TIEMPO_MAX_MS = 15000;

// =====================================================================
ACAN2517FD can(PIN_CS, SPI, PIN_INT);
// Ademas de posicion, velocidad y par, se piden corrientes, voltaje, temperaturas y potencia
Moteus moteus(can, []() {
  Moteus::Options o;
  o.id = MOTEUS_ID;
  o.query_format.q_current           = Moteus::kFloat;
  o.query_format.d_current           = Moteus::kFloat;
  o.query_format.voltage             = Moteus::kFloat;
  o.query_format.temperature         = Moteus::kFloat;
  o.query_format.motor_temperature   = Moteus::kFloat;
  o.query_format.power               = Moteus::kFloat;
  o.query_format.trajectory_complete = Moteus::kInt8;
  return o;
}());
Moteus::PositionMode::Format formato;
Preferences memoria;

// ---------- Estado del brushless ----------
bool canOk = false;
double ceroVueltas = 0.0;
int    signo = 1;
double parGravedad = PAR_GRAVEDAD_DEFECTO;
double torqueMax = 30.0;
bool   calibradoCero = false, calibradoSentido = false;
enum Modo { DETENIDO, POSICION };
Modo modo = DETENIDO;
bool ceroVerificado = false;
double destinoGrados = 0.0, ffActual = 0.0;
uint32_t ultimoComando = 0, ultimaGrafica = 0, ultimaTelemetria = 0;
uint32_t enviados = 0, respondidos = 0;
bool graficando = false, telemetria = false;
bool reportePendiente = false;
uint32_t inicioMovimiento = 0, inicioQuieto = 0;
bool midiendoGravedad = false;
uint32_t inicioMedicion = 0;
double sumaPar = 0; int muestrasPar = 0;

// ---------- Estado de los servos y encoders ----------
struct EstadoPar {
  bool activo, moviendo, encResponde, encAlineado, reporte, posConocida;
  uint32_t tLlegada;
  float destino, comando, escrito, encGrados, ceroGrados, vel;
  uint16_t raw, ceroRaw;
  uint8_t estadoIman;
};
EstadoPar par[2];
bool hayPCA = false, hayTCA = false;
float ceroCorriente[3] = {2.5f, 2.5f, 2.5f}, corriente[3] = {0, 0, 0};

void tareasServos();   // se usa dentro de las esperas del brushless

// Perfil suave: acelera, va a velocidad constante y frena antes de llegar.
// Devuelve true cuando llego al destino.
bool perfilSuave(float &pos, float &vel, float destino, float vmax, float acel, float dt) {
  const float falta = destino - pos;
  if (fabsf(falta) < 0.05f && fabsf(vel) <= acel * dt) { pos = destino; vel = 0; return true; }
  const float vFreno = sqrtf(2.0f * acel * fabsf(falta));          // velocidad maxima para poder frenar a tiempo
  const float vObjetivo = copysignf(fminf(vmax, vFreno), falta);
  if (vel < vObjetivo) vel = fminf(vObjetivo, vel + acel * dt);
  else                 vel = fmaxf(vObjetivo, vel - acel * dt);
  const float nuevo = pos + vel * dt;
  if ((destino - nuevo) * falta <= 0) { pos = destino; vel = 0; return true; }
  pos = nuevo;
  return false;
}


// =====================================================================
// BRUSHLESS
// =====================================================================
double motorAGrados(double vueltas) { return ANG_COLGANDO + signo * (vueltas - ceroVueltas) * 360.0; }
double gradosAMotor(double grados)  { return ceroVueltas + signo * (grados - ANG_COLGANDO) / 360.0; }
// Par por gravedad: maximo con la pata horizontal (0), cero colgando (-90) y vertical arriba (90)
double parCompensacion(double gradosPata) { return signo * parGravedad * cos(gradosPata * DEG_TO_RAD); }

void guardar() {
  memoria.putDouble("cero", ceroVueltas);
  memoria.putInt("signo", signo);
  memoria.putDouble("grav", parGravedad);
  memoria.putBool("okCero", calibradoCero);
  memoria.putBool("okSent", calibradoSentido);
  memoria.putDouble("tmax", torqueMax);
}

// Promedia la posicion ~0,5 s con el motor libre (los servos siguen trabajando)
double promedioPosicion() {
  double suma = 0; int n = 0;
  for (int i = 0; i < 25; i++) {
    moteus.SetStop();
    suma += moteus.last_result().values.position; n++;
    uint32_t t0 = millis();
    while (millis() - t0 < 20) tareasServos();
  }
  return suma / n;
}

void mostrarCalibracion() {
  Serial.println("Calibracion brushless (ESP32):");
  Serial.printf("  Cero:     %.4f vueltas (%s)\n", ceroVueltas, calibradoCero ? "OK" : "FALTA");
  Serial.printf("  Sentido:  signo %d (%s)\n", signo, calibradoSentido ? "OK" : "FALTA");
  Serial.printf("  Gravedad: %.2f Nm en la horizontal (0 grados)\n", parGravedad);
  Serial.printf("  Par max:  %.1f Nm\n", torqueMax);
  Serial.println("Moteus (tview): kp 200, kd 4, ki 50, ilimit 12, 6 A");
  for (int i = 0; i < 2; i++)
    Serial.printf("Encoder %s: %s (cero raw %u = %.1f grados)\n", CFG[i].nombre,
                  par[i].encAlineado ? "alineado" : "SIN alinear", par[i].ceroRaw, par[i].ceroGrados);
}

void imprimirReporte(const char *titulo) {
  const auto &v = moteus.last_result().values;
  const double pos = motorAGrados(v.position);
  Serial.println(titulo);
  Serial.printf("  Destino:  %.1f grados\n", destinoGrados);
  Serial.printf("  Posicion: %.2f grados\n", pos);
  Serial.printf("  Error:    %.2f grados\n", destinoGrados - pos);
  Serial.printf("  Par:      %.2f Nm\n", v.torque);
  Serial.printf("  Fault:    %d\n", (int)v.fault);
  Serial.printf("  Tiempo:   %.1f s\n", (millis() - inicioMovimiento) / 1000.0);
}

void verificarCeroArranque() {
  Serial.println("--- Verificacion del cero (brushless) ---");
  if (!calibradoCero || !calibradoSentido) {
    Serial.println("Sin calibracion guardada.");
    Serial.println("Pata colgando: z. Pata horizontal: h.");
    return;
  }
  const double ang = motorAGrados(promedioPosicion());
  Serial.printf("  Pata colgando: %.2f grados (debe ser -90)\n", ang);
  if (fabs(ang - ANG_COLGANDO) <= TOL_CERO) {
    Serial.println("  El cero coincide.");
    Serial.println("  Confirma con el transportador y envia v.");
  } else {
    Serial.println("  ATENCION: el cero NO coincide.");
    Serial.println("  Con la pata colgando quieta, envia z.");
  }
}

bool brushlessDisponible() {
  if (!canOk) Serial.println("Brushless no disponible: el MCP2518FD no inicio (revisa SPI y 3V3).");
  return canOk;
}

void irA(double grados) {
  if (!brushlessDisponible()) return;
  if (!calibradoCero || !calibradoSentido) { Serial.println("Primero calibra: s, z (colgando) y h (horizontal)."); return; }
  if (!ceroVerificado) { Serial.println("Verifica el cero primero (v o z)."); return; }
  if (grados < ANG_MIN || grados > ANG_MAX) {
    Serial.println("Angulo fuera de rango.");
    Serial.printf("  Usa un valor entre %.0f y %.0f.\n", ANG_MIN, ANG_MAX);
    return;
  }
  destinoGrados = grados;
  modo = POSICION;
  reportePendiente = true; inicioMovimiento = millis(); inicioQuieto = 0;
  Serial.printf("Ir a %.1f grados\n", destinoGrados);
}

void tareasBrushless() {
  if (!canOk) return;
  if (millis() - ultimoComando >= PERIODO_MS) {
    ultimoComando = millis();
    enviados++;
    bool respuesta;
    if (modo == DETENIDO) {
      ffActual = 0.0;
      respuesta = moteus.SetStop();
    } else {
      Moteus::PositionMode::Command cmd;
      const double actual = motorAGrados(moteus.last_result().values.position);
      cmd.position           = gradosAMotor(destinoGrados);
      cmd.velocity           = 0.0;
      ffActual               = midiendoGravedad ? 0.0 : parCompensacion(actual);
      cmd.feedforward_torque = ffActual;
      cmd.maximum_torque     = torqueMax;
      cmd.velocity_limit     = VEL_LIMITE;
      cmd.accel_limit        = ACEL_LIMITE;
      respuesta = moteus.SetPosition(cmd, &formato);
    }
    if (respuesta) respondidos++;
  }

  if (midiendoGravedad) {
    const uint32_t t = millis() - inicioMedicion;
    if (t > 1500 && t <= 3000) { sumaPar += moteus.last_result().values.torque; muestrasPar++; }
    if (t > 3000) {
      midiendoGravedad = false;
      if (muestrasPar > 0) { parGravedad = fabs(sumaPar / muestrasPar); guardar(); }
      Serial.printf("Gravedad guardada: %.2f Nm\n", parGravedad);
    }
  }

  if (reportePendiente && modo == POSICION) {
    const auto &v = moteus.last_result().values;
    const bool cerca = fabs(destinoGrados - motorAGrados(v.position)) < TOL_LLEGADA && fabs(v.velocity) < 0.005;
    if (cerca) {
      if (inicioQuieto == 0) inicioQuieto = millis();
      if (millis() - inicioQuieto > 500) { imprimirReporte("Llego:"); reportePendiente = false; }
    } else {
      inicioQuieto = 0;
    }
    if (reportePendiente && millis() - inicioMovimiento > TIEMPO_MAX_MS) {
      imprimirReporte("No llego. Revisa:");
      reportePendiente = false;
    }
  } else if (modo != POSICION) {
    reportePendiente = false;
  }
}

// =====================================================================
// SERVOS Y ENCODERS (I2C)
// =====================================================================
bool existeI2C(uint8_t dir) {
  Wire.beginTransmission(dir);
  return Wire.endTransmission() == 0;
}

// ---- PCA9685 por registros (el mismo codigo del diagnostico, que ya se probo) ----
const float PCA_OSC_HZ = 26579680.0f;
uint8_t pcaPrescale = 131;

bool pcaEscribir(uint8_t reg, uint8_t valor) {
  Wire.beginTransmission(DIR_PCA9685);
  Wire.write(reg); Wire.write(valor);
  return Wire.endTransmission() == 0;
}

bool pcaLeer(uint8_t reg, uint8_t &valor) {
  Wire.beginTransmission(DIR_PCA9685);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)DIR_PCA9685, 1) != 1) return false;
  valor = Wire.read();
  return true;
}

bool pcaSalida(uint8_t ch, uint16_t on, uint16_t off) {
  Wire.beginTransmission(DIR_PCA9685);
  Wire.write(0x06 + 4 * ch);
  Wire.write(on & 0xFF); Wire.write(on >> 8);
  Wire.write(off & 0xFF); Wire.write(off >> 8);
  return Wire.endTransmission() == 0;
}

bool pcaPulsoUs(uint8_t ch, float us) {
  const float periodoUs = (pcaPrescale + 1) * 4096.0f / PCA_OSC_HZ * 1e6f;
  return pcaSalida(ch, 0, (uint16_t)(us / periodoUs * 4096.0f + 0.5f));
}

bool pcaApagar(uint8_t ch) { return pcaSalida(ch, 0, 0x1000); }   // "full off": servo sin fuerza

// 50 Hz, auto incremento, sin "All Call" (0x70 es del TCA9548A), salidas totem pole y apagadas
bool pcaIniciar() {
  pcaPrescale = (uint8_t)(PCA_OSC_HZ / (4096.0f * 50.0f) + 0.5f) - 1;
  bool ok = pcaEscribir(0x00, 0x10);          // dormir para cambiar la frecuencia
  ok &= pcaEscribir(0xFE, pcaPrescale);
  ok &= pcaEscribir(0x00, 0x20);              // despertar, auto incremento, sin All Call
  delay(2);
  ok &= pcaEscribir(0x00, 0xA0);              // restart
  ok &= pcaEscribir(0x01, 0x04);              // totem pole
  ok &= pcaEscribir(0xFD, 0x10);              // todas las salidas apagadas
  uint8_t m1 = 0, pre = 0;
  ok &= pcaLeer(0x00, m1) && pcaLeer(0xFE, pre);
  Serial.printf("PCA9685: MODE1 0x%02X, prescale %u (%.1f Hz) %s\n", m1, pre,
                PCA_OSC_HZ / (4096.0f * (pre + 1)), ok && pre == pcaPrescale && !(m1 & 0x10) ? "OK" : "<- REVISAR");
  return ok;
}

bool canalTCA(uint8_t canal) {
  Wire.beginTransmission(DIR_TCA9548A);
  Wire.write(1 << canal);
  return Wire.endTransmission() == 0;
}

// Lee STATUS (0x0B) y RAW ANGLE (0x0C-0x0D) del AS5600 en un canal del TCA9548A
bool leerAS5600(uint8_t canal, uint16_t &raw, uint8_t &estado) {
  if (!hayTCA || !canalTCA(canal)) return false;
  Wire.beginTransmission(DIR_AS5600);
  Wire.write(0x0B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)DIR_AS5600, 3) != 3) return false;
  estado = Wire.read();
  const uint8_t alto = Wire.read(), bajo = Wire.read();
  raw = ((uint16_t)(alto & 0x0F) << 8) | bajo;
  return true;
}

float encoderAGrados(int i) {
  int d = (int)par[i].raw - (int)par[i].ceroRaw;
  if (d > 2048) d -= 4096;
  if (d < -2048) d += 4096;
  return par[i].ceroGrados + CFG[i].signoEncoder * d * 360.0f / 4096.0f;
}

// g en grados del servo (0 a 180). Los comandos usan angulo con signo: servo = 90 + angulo
uint16_t gradosAUs(float g) {
  g = constrain(g, 0.0f, 180.0f);
  return (uint16_t)(SERVO_MIN_US + g / 180.0f * (SERVO_MAX_US - SERVO_MIN_US));
}

void escribirPar(int i) {
  const ConfigPar &c = CFG[i];
  const float g = 90.0f + par[i].comando;          // angulo con signo -> grados del servo
  // los dos servos del par reciben siempre el mismo angulo (sin espejo)
  const bool ok = pcaPulsoUs(c.canal1, gradosAUs(g) + c.trim1_us) & pcaPulsoUs(c.canal2, gradosAUs(g) + c.trim2_us);
  if (!ok) Serial.printf("Error I2C al escribir los servos %s\n", c.nombre);
  par[i].escrito = g;
}

// Guarda la ultima posicion del par en la memoria del ESP32 (sobrevive al apagado)
void guardarPosicion(int i) {
  char k[4] = {CFG[i].letra, 'Q', 0, 0};
  memoria.putDouble(k, par[i].comando);
  k[1] = 'J'; memoria.putBool(k, true);
  par[i].posConocida = true;
}

void soltarPar(int i, bool avisar) {
  if (par[i].activo) guardarPosicion(i);
  if (hayPCA) { pcaApagar(CFG[i].canal1); pcaApagar(CFG[i].canal2); }
  par[i].activo = false; par[i].moviendo = false; par[i].reporte = false; par[i].vel = 0;
  if (avisar) Serial.printf("Servos %s sueltos (sin fuerza).\n", CFG[i].nombre);
}

void moverPar(int i, float g) {
  const ConfigPar &c = CFG[i];
  if (!hayPCA) { Serial.println("Servos no disponibles: no se encontro el PCA9685 (0x40)."); return; }
  if (g < c.minimo || g > c.maximo) {
    Serial.printf("Servos %s: angulo fuera de rango.\n", c.nombre);
    Serial.printf("  Usa un valor entre %.0f y %.0f.\n", c.minimo, c.maximo);
    return;
  }
  EstadoPar &p = par[i];
  if (!p.activo) {
    // Arrancar desde donde esta el servo para que el movimiento sea suave desde el principio:
    // 1) encoder alineado (posicion real)  2) ultima posicion guardada  3) sin datos: va directo
    if (p.encAlineado && p.encResponde) {
      p.comando = constrain(p.encGrados, c.minimo, c.maximo);
    } else if (p.posConocida) {
      Serial.printf("  %s: arranca suave desde la ultima posicion guardada (%.1f grados).\n", c.nombre, p.comando);
    } else {
      p.comando = g;
      Serial.printf("  %s: sin posicion conocida, esta unica vez va directo.\n", c.nombre);
    }
    p.activo = true; p.vel = 0;
    escribirPar(i);
  }
  p.destino = g; p.moviendo = true;
  Serial.printf("Servos %s a %.1f grados\n", c.nombre, g);
}

void alinearEncoder(int i) {
  EstadoPar &p = par[i];
  if (!p.activo || p.moviendo) { Serial.printf("Mueve los servos %s a una posicion conocida\n", CFG[i].nombre); Serial.println("y espera que lleguen antes de alinear."); return; }
  if (!p.encResponde) { Serial.printf("El encoder %s no responde (canal %d del TCA9548A).\n", CFG[i].nombre, CFG[i].canalEncoder); return; }
  p.ceroRaw = p.raw; p.ceroGrados = p.comando; p.encAlineado = true;
  char k[4] = {CFG[i].letra, 'r', 0, 0};
  memoria.putInt(k, p.ceroRaw);
  k[1] = 'g'; memoria.putDouble(k, p.ceroGrados);
  k[1] = 'l'; memoria.putBool(k, true);
  Serial.printf("Encoder %s alineado: raw %u = %.1f grados\n", CFG[i].nombre, p.ceroRaw, p.ceroGrados);
}

void reporteServos(int i) {
  const EstadoPar &p = par[i];
  Serial.printf("Servos %s llegaron:\n", CFG[i].nombre);
  Serial.printf("  Comando:  %.1f grados\n", p.comando);
  if (p.encAlineado && p.encResponde) {
    Serial.printf("  Encoder:  %.1f grados\n", p.encGrados);
    Serial.printf("  Error:    %.1f grados\n", p.comando - p.encGrados);
  } else {
    Serial.println("  Encoder:  sin alinear (usa az o bz)");
  }
}

void ceroCorrientes() {
  for (int k = 0; k < 3; k++) {
    uint32_t suma = 0;
    for (int n = 0; n < 64; n++) suma += analogReadMilliVolts(PIN_CORRIENTE[k]);
    ceroCorriente[k] = suma / 64.0f / 1000.0f * DIVISOR_CORRIENTE;
    corriente[k] = 0;
  }
}

void leerCorrientes() {
  for (int k = 0; k < 3; k++) {
    const float v = analogReadMilliVolts(PIN_CORRIENTE[k]) / 1000.0f * DIVISOR_CORRIENTE;
    const float a = (v - ceroCorriente[k]) / SENS_V_POR_A;
    corriente[k] += 0.2f * (a - corriente[k]);   // filtro suave
  }
}

void tareasServos() {
  static uint32_t ultimo = 0;
  const uint32_t ahora = millis();
  if (ahora - ultimo < PERIODO_SERVOS_MS) return;
  float dt = (ahora - ultimo) / 1000.0f;
  if (ultimo == 0 || dt > 0.1f) dt = PERIODO_SERVOS_MS / 1000.0f;
  ultimo = ahora;

  for (int i = 0; i < 2; i++) {
    EstadoPar &p = par[i];
    uint16_t raw; uint8_t est;
    p.encResponde = leerAS5600(CFG[i].canalEncoder, raw, est);
    if (p.encResponde) { p.raw = raw; p.estadoIman = est; p.encGrados = encoderAGrados(i); }
    if (!p.activo) continue;
    const bool llego = perfilSuave(p.comando, p.vel, p.destino, CFG[i].velocidad, ACEL_SERVOS, dt);
    if (llego && p.moviendo) { p.moviendo = false; p.reporte = true; p.tLlegada = ahora; escribirPar(i); guardarPosicion(i); }
    if (p.comando != p.escrito) escribirPar(i);
    // reporte 0,4 s despues de llegar, para que el servo real se asiente y el encoder lo muestre
    if (p.reporte && ahora - p.tLlegada > 400) { p.reporte = false; reporteServos(i); }
  }
  leerCorrientes();
}

void imprimirEstado() {
  Serial.println("Estado brushless:");
  if (canOk) {
    const auto &v = moteus.last_result().values;
    Serial.printf("  Modo:     %d   Fault: %d\n", (int)v.mode, (int)v.fault);
    Serial.printf("  Pata:     %.2f grados\n", motorAGrados(v.position));
    Serial.printf("  Par:      %.2f Nm\n", v.torque);
    Serial.printf("  Bateria:  %.1f V   Temp: %.1f C\n", v.voltage, v.temperature);
    Serial.printf("  CAN:      %lu/%lu\n", (unsigned long)respondidos, (unsigned long)enviados);
  } else {
    Serial.println("  No disponible (MCP2518FD sin iniciar).");
  }
  for (int i = 0; i < 2; i++) {
    const EstadoPar &p = par[i];
    Serial.printf("Servos %s: %s\n", CFG[i].nombre, !hayPCA ? "sin PCA9685" : p.activo ? (p.moviendo ? "moviendose" : "quietos") : "sueltos");
    if (p.activo) Serial.printf("  Destino %.1f  Comando %.1f grados\n", p.destino, p.comando);
    if (p.encResponde)
      Serial.printf("  Encoder raw %u  %s  iman %s\n", p.raw, p.encAlineado ? "alineado" : "sin alinear",
                    (p.estadoIman & 0x20) ? ((p.estadoIman & 0x10) ? "debil" : (p.estadoIman & 0x08) ? "fuerte" : "OK") : "NO detectado");
    else
      Serial.printf("  Encoder no responde (canal %d)\n", CFG[i].canalEncoder);
  }
  Serial.printf("Corrientes: %.2f  %.2f  %.2f A\n", corriente[0], corriente[1], corriente[2]);
}

// =====================================================================
// COMANDOS
// =====================================================================
void pararTodo() {
  modo = DETENIDO;
  soltarPar(0, false); soltarPar(1, false);
  Serial.println("PARAR TODO: brushless libre y servos sueltos.");
}

void comandoServos(int i, String txt) {
  const char c1 = tolower(txt.charAt(1));
  if (c1 == 'x') { soltarPar(i, true); return; }
  if (c1 == 'z') { alinearEncoder(i); return; }
  if (c1 == 'v' && txt.length() > 2) {
    const float v = txt.substring(2).toFloat();
    if (v >= 5 && v <= 360) { CFG[i].velocidad = v; Serial.printf("Velocidad servos %s: %.0f grados/s\n", CFG[i].nombre, v); }
    else Serial.println("Usa una velocidad entre 5 y 360 (ej: av60)");
    return;
  }
  if (isDigit(c1) || c1 == '.' || c1 == '-') { moverPar(i, txt.substring(1).toFloat()); return; }
  Serial.printf("Comando no reconocido: %s\n", txt.c_str());
}

void procesarComando(char c) {
  switch (c) {
    case 's': modo = DETENIDO; Serial.println("Stop brushless: motor libre (sosten la pata)."); break;
    case 'x': pararTodo(); break;
    case 'v':
      if (!brushlessDisponible()) break;
      if (!calibradoCero || !calibradoSentido) { Serial.println("No hay calibracion: usa z y h."); break; }
      ceroVerificado = true; Serial.println("Cero confirmado. Ya puedes enviar angulos."); break;
    case 'z':
      if (!brushlessDisponible()) break;
      if (modo != DETENIDO) { Serial.println("Envia s primero y deja"); Serial.println("la pata colgando quieta."); break; }
      ceroVueltas = promedioPosicion(); calibradoCero = true; ceroVerificado = true; guardar();
      Serial.println("CERO guardado (pata colgando = -90 grados):");
      Serial.printf("  %.4f vueltas del motor\n", ceroVueltas);
      break;
    case 'h': {
      if (!brushlessDisponible()) break;
      if (modo != DETENIDO || !calibradoCero) { Serial.println("Primero: s y z con la pata colgando."); Serial.println("Luego pata horizontal y h."); break; }
      const double delta = (promedioPosicion() - ceroVueltas) * 360.0;
      if (fabs(delta) < 30) { Serial.println("La pata casi no se movio."); Serial.println("Llevala a la horizontal y envia h."); break; }
      signo = (delta > 0) ? 1 : -1; calibradoSentido = true; guardar();
      Serial.printf("SENTIDO guardado: signo %d\n", signo);
      Serial.printf("  Recorrido medido: %.1f grados (debe ser ~90, de colgando a horizontal)\n", fabs(delta));
      break;
    }
    case 'm':
      if (!brushlessDisponible()) break;
      if (modo != POSICION || fabs(destinoGrados) > 1) { Serial.println("Envia 0 (horizontal), espera que"); Serial.println("quede quieta y luego envia m."); break; }
      midiendoGravedad = true; inicioMedicion = millis(); sumaPar = 0; muestrasPar = 0;
      Serial.println("Midiendo el par (3 s)."); Serial.println("No toques la pata...");
      break;
    case 'e': imprimirEstado(); break;
    case 'c': mostrarCalibracion(); break;
    case 'i': ceroCorrientes(); Serial.println("Sensores de corriente en cero."); break;
    case 'r':
      calibradoCero = calibradoSentido = ceroVerificado = false; signo = 1; ceroVueltas = 0;
      parGravedad = PAR_GRAVEDAD_DEFECTO; modo = DETENIDO; guardar();
      Serial.println("Calibracion del brushless borrada."); break;
    case 'g': graficando = !graficando; Serial.println(graficando ? "Graficas activadas" : "Graficas en pausa"); break;
    default: Serial.printf("Comando no reconocido: %c\n", c); break;
  }
}

String linea = "";
uint32_t ultimoCaracter = 0;

void procesarLinea(String txt) {
  txt.trim();
  if (txt.length() == 0) return;
  const char c0 = tolower(txt.charAt(0));
  if (isDigit(c0) || c0 == '.' || c0 == '-') {
    irA(txt.toFloat());
  } else if ((c0 == 'a' || c0 == 'b') && txt.length() > 1) {
    comandoServos(c0 == 'a' ? 0 : 1, txt);
  } else if (c0 == 't' && txt.length() > 1) {
    const double t = txt.substring(1).toFloat();
    if (t > 0 && t <= TORQUE_TOPE) { torqueMax = t; guardar(); Serial.printf("Limite de par: %.1f Nm\n", torqueMax); }
    else Serial.printf("Usa un valor entre 0 y %.0f (ej: t30)\n", TORQUE_TOPE);
  } else if (c0 == 'p' && txt.length() == 2) {
    telemetria = (txt.charAt(1) == '1');
    Serial.println(telemetria ? "Telemetria activada" : "Telemetria en pausa");
  } else if (txt.length() == 1) {
    procesarComando(c0);
  } else {
    Serial.printf("Comando no reconocido: %s\n", txt.c_str());
  }
}

void leerSerie() {
  while (Serial.available()) {
    const char c = Serial.read();
    if (c == '\n' || c == '\r') { procesarLinea(linea); linea = ""; }
    else { linea += c; ultimoCaracter = millis(); }
  }
  if (linea.length() > 0 && millis() - ultimoCaracter > 300) { procesarLinea(linea); linea = ""; }
}

// =====================================================================
// TELEMETRIA
// =====================================================================
void enviarGrafica() {
  const auto &v = moteus.last_result().values;
  const double pos = motorAGrados(v.position);
  const double dest = (modo == POSICION) ? destinoGrados : pos;
  Serial.printf(">destino_grados:%.2f\n", dest);
  Serial.printf(">posicion_grados:%.2f\n", pos);
  Serial.printf(">par_Nm:%.3f\n", v.torque);
  Serial.printf(">servo_arriba:%.1f\n", par[0].comando);
  Serial.printf(">servo_abajo:%.1f\n", par[1].comando);
}

void enviarTelemetria() {
  if (canOk) {
    const auto &v = moteus.last_result().values;
    const double pos = motorAGrados(v.position);
    const int banderas = (calibradoCero ? 1 : 0) | (calibradoSentido ? 2 : 0) | (ceroVerificado ? 4 : 0) |
                         (modo == POSICION ? 8 : 0) | (midiendoGravedad ? 16 : 0) |
                         (v.trajectory_complete ? 32 : 0) | (reportePendiente ? 64 : 0);
    Serial.printf("@T,%lu,%d,%d,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,%.2f,%.1f,%.1f,%.2f,%.1f,%.2f,%d,%lu,%lu,%.4f\n",
                  (unsigned long)millis(), (int)v.mode, (int)v.fault, pos, destinoGrados,
                  signo * v.velocity * 360.0, signo * v.torque, signo * ffActual, v.q_current, v.d_current,
                  v.voltage, v.temperature, v.motor_temperature, v.power, torqueMax, parGravedad,
                  banderas, (unsigned long)respondidos, (unsigned long)enviados, v.position);
  }
  Serial.printf("@S,%lu", (unsigned long)millis());
  for (int i = 0; i < 2; i++) {
    const EstadoPar &p = par[i];
    const int b = (p.activo ? 1 : 0) | (p.encResponde ? 2 : 0) | ((p.estadoIman & 0x20) ? 4 : 0) |
                  ((p.estadoIman & 0x10) ? 8 : 0) | ((p.estadoIman & 0x08) ? 16 : 0) |
                  (p.encAlineado ? 32 : 0) | (p.moviendo ? 64 : 0);
    const float enc = (p.encAlineado && p.encResponde) ? p.encGrados : NAN;
    Serial.printf(",%.1f,%.1f,%.1f,%u,%d", p.destino, p.comando, enc, p.raw, b);
  }
  Serial.printf(",%.2f,%.2f,%.2f\n", corriente[0], corriente[1], corriente[2]);
}

// =====================================================================
void setup() {
  Serial.setTxBufferSize(1024);
  Serial.begin(115200);
  delay(500);
  Serial.println("Pata: brushless (moteus-c1) + servos (PCA9685) + encoders (AS5600)");

  memoria.begin("pata", false);
  ceroVueltas      = memoria.getDouble("cero", 0.0);
  signo            = memoria.getInt("signo", 1);
  parGravedad      = memoria.getDouble("grav", PAR_GRAVEDAD_DEFECTO);
  calibradoCero    = memoria.getBool("okCero", false);
  calibradoSentido = memoria.getBool("okSent", false);
  torqueMax        = memoria.getDouble("tmax", torqueMax);
  for (int i = 0; i < 2; i++) {
    char k[4] = {CFG[i].letra, 'r', 0, 0};
    par[i].ceroRaw = memoria.getInt(k, 0);
    k[1] = 'g'; par[i].ceroGrados = memoria.getDouble(k, 0.0);
    k[1] = 'l'; par[i].encAlineado = memoria.getBool(k, false);
    k[1] = 'Q'; par[i].comando = memoria.getDouble(k, 0.0);
    k[1] = 'J'; par[i].posConocida = memoria.getBool(k, false);
  }

  // --- I2C: servos y encoders ---
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);       // igual que el diagnostico: mas tolerante con cables largos
  Wire.setTimeOut(5);
  hayPCA = existeI2C(DIR_PCA9685);
  if (hayPCA) {
    pcaIniciar();
    soltarPar(0, false); soltarPar(1, false);
    Serial.println("PCA9685 (0x40) OK: servos sueltos hasta recibir a.. o b..");
  } else {
    Serial.println("ERROR: no se encontro el PCA9685 (0x40). Revisa SDA 21, SCL 22 y su VCC.");
  }
  hayTCA = existeI2C(DIR_TCA9548A);
  if (!hayTCA) Serial.println("ERROR: no se encontro el TCA9548A (0x70). Los encoders no se leeran.");
  for (int i = 0; i < 2; i++) {
    uint16_t raw; uint8_t est;
    const bool ok = leerAS5600(CFG[i].canalEncoder, raw, est);
    if (ok) { par[i].raw = raw; par[i].estadoIman = est; par[i].encResponde = true; par[i].encGrados = encoderAGrados(i); }
    Serial.printf("Encoder %s (canal %d): %s\n", CFG[i].nombre, CFG[i].canalEncoder,
                  !ok ? "NO responde" : (est & 0x20) ? "OK, iman detectado" : "responde, pero NO detecta el iman");
  }
  analogSetAttenuation(ADC_11db);
  ceroCorrientes();

  // --- CAN: brushless ---
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI);
  ACAN2517FDSettings ajustes(CRISTAL_40MHZ ? ACAN2517FDSettings::OSC_40MHz : ACAN2517FDSettings::OSC_20MHz,
                             1000ll * 1000ll, DataBitRateFactor::x1);
  ajustes.mArbitrationSJW = 2;
  ajustes.mDriverTransmitFIFOSize = 1;
  ajustes.mDriverReceiveFIFOSize = 2;
  const uint32_t error = can.begin(ajustes, [] { can.isr(); });
  if (error != 0) {
    Serial.printf("ERROR al iniciar el MCP2518FD: 0x%lX\n", (unsigned long)error);
    Serial.println("Brushless desactivado. Los servos siguen funcionando.");
  } else {
    canOk = true;
    Serial.println("MCP2518FD OK: brushless disponible.");
    formato.velocity_limit     = Moteus::kFloat;
    formato.accel_limit        = Moteus::kFloat;
    formato.maximum_torque     = Moteus::kFloat;
    formato.feedforward_torque = Moteus::kFloat;
    moteus.SetStop();
    mostrarCalibracion();
    verificarCeroArranque();
  }

  Serial.println("Comandos:");
  Serial.println("  Brushless: -90 a 90 (0 horizontal, 90 arriba), s, v, z, h, m, t30");
  Serial.println("  Servos arriba: a0 centro, a45, a-45, ax, az, av20");
  Serial.println("  Servos abajo:  b0 centro, b45, b-45, bx, bz, bv20");
  Serial.println("  e estado   x PARAR TODO   i cero corrientes");
  Serial.println("  p1 / p0 telemetria interfaz grafica");
}

void loop() {
  leerSerie();
  tareasBrushless();
  tareasServos();

  if (graficando && millis() - ultimaGrafica >= PERIODO_GRAFICA_MS) {
    ultimaGrafica = millis();
    enviarGrafica();
  }
  if (telemetria && millis() - ultimaTelemetria >= PERIODO_TELEMETRIA_MS) {
    ultimaTelemetria = millis();
    enviarTelemetria();
  }
}