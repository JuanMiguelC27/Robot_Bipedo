/*
 * Diagnostico del PCA9685 (servos) con ESP32
 * --------------------------------------------------------------------------
 * Cargar:   pio run -e diag_pca -t upload -t monitor
 *           (en VS Code: elige el entorno "env:diag_pca" en la barra de abajo)
 * Volver al programa normal:  pio run -e esp32dev -t upload
 *
 * Conexion: GPIO 21 SDA, GPIO 22 SCL, 3V3 al VCC del PCA9685, GND comun.
 * Los servos se alimentan por V+ (bornera del PCA9685), NO por el ESP32.
 *
 * No usa librerias: habla directo con los registros del chip, para ver
 * exactamente donde esta la falla.
 *
 * COMANDOS (escribe y presiona Enter):
 *   e          escanear el bus I2C (que equipos responden)
 *   r          leer y explicar los registros del PCA9685
 *   k          prueba de escritura y lectura (comunicacion)
 *   i          reiniciar y configurar el PCA9685 a 50 Hz
 *   c<n>       elegir canal 0 a 15 (ej: c0)
 *   <numero>   pulso en microsegundos al canal elegido (500 a 2500, ej: 1500)
 *   ANGULOS: 0 = centro del servo, de -90 a 90 (ej: g0 centro, g45, g-45)
 *   g<grados>  angulo -90 a 90 al canal elegido (ej: g0, g-30)
 *   b          barrido lento 0 -> 180 -> 0 del canal elegido
 *   p          probar los 16 canales uno por uno (90 grados, 1 s cada uno)
 *   pa<grados> par de ARRIBA (canales 12 y 13, el 13 en espejo) (ej: pa0, pa50, pa-50)
 *   pb<grados> par de ABAJO  (canales 8 y 9, el 9 en espejo) (ej: pb-30)
 *   qa<grados> / qb<grados>  igual que pa/pb pero SIN espejo (los dos al mismo angulo)
 *   v<grados/s> velocidad de los movimientos suaves (ej: v10; por defecto 20)
 *   x          apagar todas las salidas (servos sin fuerza)
 *   (g, pa, pb, qa y qb se mueven suave: aceleran, avanzan y frenan. Un pulso en us va directo.)
 *   m          medir el pulso real: une la senal (S) del canal elegido al GPIO 4
 *   t          revisar los canales del TCA9548A (donde estan los AS5600)
 *   h          ayuda
 */
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <Preferences.h>

const uint8_t PIN_SDA = 21, PIN_SCL = 22, PIN_MEDIR = 4;
const uint8_t DIR_PCA = 0x40, DIR_TCA = 0x70, DIR_AS5600 = 0x36;
const float OSC_HZ = 27000000.0f;     // el mismo valor que usa el programa principal
const float FREC_HZ = 50.0f;

// Registros del PCA9685
const uint8_t MODE1 = 0x00, MODE2 = 0x01, LED0 = 0x06, ALL_LED_OFF_H = 0xFD, PRESCALE = 0xFE;

int canal = 8;
float velocidad = 20.0f;          // grados por segundo
const float ACELERACION = 40.0f;  // grados por segundo al cuadrado (menor = mas suave)
float actual[16], objetivo[16], velCanal[16];   // NAN = posicion desconocida / sin destino

uint8_t prescale = 0;

// ---------------------------------------------------------------- bajo nivel
bool escribir(uint8_t reg, uint8_t valor) {
  Wire.beginTransmission(DIR_PCA);
  Wire.write(reg); Wire.write(valor);
  return Wire.endTransmission() == 0;
}

bool leer(uint8_t reg, uint8_t &valor) {
  Wire.beginTransmission(DIR_PCA);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)DIR_PCA, 1) != 1) return false;
  valor = Wire.read();
  return true;
}

bool existe(uint8_t dir) {
  Wire.beginTransmission(dir);
  return Wire.endTransmission() == 0;
}

float periodoUs() { return (prescale + 1) * 4096.0f / OSC_HZ * 1e6f; }

bool salida(int ch, uint16_t on, uint16_t off) {
  Wire.beginTransmission(DIR_PCA);
  Wire.write(LED0 + 4 * ch);
  Wire.write(on & 0xFF); Wire.write(on >> 8);
  Wire.write(off & 0xFF); Wire.write(off >> 8);
  return Wire.endTransmission() == 0;
}

bool pulsoUs(int ch, float us) {
  const uint16_t ticks = (uint16_t)(us / periodoUs() * 4096.0f + 0.5f);
  return salida(ch, 0, ticks);
}

bool apagar(int ch) { return salida(ch, 0, 0x1000); }          // bit "full off"
bool apagarTodo() { return escribir(ALL_LED_OFF_H, 0x10); }
float gradosAUs(float g) { return 500.0f + g / 180.0f * 2000.0f; }

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

// La ultima posicion de cada canal se guarda en la memoria del ESP32 (sobrevive al apagado),
// asi el primer movimiento despues de encender tambien es suave.
Preferences memoria;

void guardarPos(int ch) {
  char k[5]; snprintf(k, sizeof k, "c%d", ch);
  memoria.putDouble(k, actual[ch]);
}

// El servo quedo en una posicion conocida sin movimiento suave (pulso directo, barrido, prueba)
void fijar(int ch, float g) { objetivo[ch] = NAN; velCanal[ch] = 0; actual[ch] = g; guardarPos(ch); }

// Detener el movimiento suave y recordar donde quedo
void detener(int ch) {
  if (!isnan(objetivo[ch]) && !isnan(actual[ch])) guardarPos(ch);
  objetivo[ch] = NAN; velCanal[ch] = 0;
}

// Mover suave un canal desde su ultima posicion conocida
void irSuave(int ch, float g) {
  if (isnan(actual[ch])) {
    fijar(ch, g);
    pulsoUs(ch, gradosAUs(g));
    Serial.printf("  Canal %d: sin posicion guardada, esta unica vez va directo a %+.0f\n", ch, g - 90);
    return;
  }
  objetivo[ch] = g;
}

void tareasSuaves() {
  static uint32_t ultimo = 0;
  const uint32_t ahora = millis();
  if (ahora - ultimo < 20) return;
  const float dt = ultimo == 0 ? 0.02f : fminf(0.1f, (ahora - ultimo) / 1000.0f);
  ultimo = ahora;
  for (int ch = 0; ch < 16; ch++) {
    if (isnan(objetivo[ch])) continue;
    const bool llego = perfilSuave(actual[ch], velCanal[ch], objetivo[ch], velocidad, ACELERACION, dt);
    pulsoUs(ch, gradosAUs(actual[ch]));
    if (llego) { Serial.printf("  Canal %d llego a %+.0f grados\n", ch, actual[ch] - 90); objetivo[ch] = NAN; guardarPos(ch); }
  }
}

// ---------------------------------------------------------------- pruebas
void escanear() {
  Serial.println("Escaneando I2C (SDA 21, SCL 22)...");
  int n = 0;
  for (uint8_t d = 1; d < 127; d++) {
    if (!existe(d)) continue;
    n++;
    const char *nombre = "desconocido";
    if (d == 0x40) nombre = "PCA9685 (servos)";
    else if (d == 0x70) nombre = "TCA9548A (o el All Call del PCA9685)";
    else if (d == 0x4A || d == 0x4B) nombre = "BNO085 (IMU)";
    else if (d == 0x36) nombre = "AS5600 conectado directo";
    else if (d >= 0x41 && d <= 0x7F && d != 0x70) nombre = "otro PCA9685 o equipo con direccion cambiada";
    Serial.printf("  0x%02X  %s\n", d, nombre);
  }
  if (n == 0) {
    Serial.println("  Nadie responde. Revisa:");
    Serial.println("  - SDA en GPIO 21 y SCL en GPIO 22 (no invertidos)");
    Serial.println("  - VCC del PCA9685 a 3V3 y GND comun con el ESP32");
  } else if (!existe(DIR_PCA)) {
    Serial.println("  El PCA9685 (0x40) NO responde. Revisa su VCC y los puentes A0-A5.");
  }
}

void explicarRegistros() {
  uint8_t m1, m2, pre;
  if (!leer(MODE1, m1) || !leer(MODE2, m2) || !leer(PRESCALE, pre)) {
    Serial.println("No se pudieron leer los registros: el PCA9685 no responde.");
    return;
  }
  Serial.printf("MODE1 = 0x%02X\n", m1);
  Serial.printf("  SLEEP    %d  %s\n", (m1 >> 4) & 1, (m1 & 0x10) ? "<- dormido: NO genera pulsos (usa i)" : "despierto, OK");
  Serial.printf("  AI       %d  auto incremento\n", (m1 >> 5) & 1);
  Serial.printf("  EXTCLK   %d  %s\n", (m1 >> 6) & 1, (m1 & 0x40) ? "<- reloj externo (raro en modulos)" : "reloj interno, OK");
  Serial.printf("  RESTART  %d\n", (m1 >> 7) & 1);
  Serial.printf("  ALLCALL  %d  %s\n", m1 & 1, (m1 & 1) ? "<- responde en 0x70, choca con el TCA9548A (usa i)" : "apagado, OK");
  Serial.printf("MODE2 = 0x%02X\n", m2);
  Serial.printf("  OUTDRV   %d  %s\n", (m2 >> 2) & 1, (m2 & 0x04) ? "totem pole, OK para servos" : "drenaje abierto");
  Serial.printf("  INVRT    %d  %s\n", (m2 >> 4) & 1, (m2 & 0x10) ? "<- salidas invertidas (mal para servos)" : "normal, OK");
  const float f = OSC_HZ / (4096.0f * (pre + 1));
  Serial.printf("PRESCALE = %u  ->  %.1f Hz %s\n", pre, f, (f > 45 && f < 55) ? "(OK para servos)" : "<- no es 50 Hz (usa i)");
}

void pruebaComunicacion() {
  // Escribe valores en el registro ON del canal 15 y los vuelve a leer
  const uint8_t reg = LED0 + 4 * 15;
  uint8_t antes = 0, leido = 0;
  if (!leer(reg, antes)) { Serial.println("FALLA: no se puede leer el PCA9685."); return; }
  int errores = 0;
  for (uint8_t v : {0x55, 0xAA, 0x0F, 0xF0}) {
    if (!escribir(reg, v) || !leer(reg, leido) || leido != v) {
      errores++;
      Serial.printf("  escribi 0x%02X y lei 0x%02X\n", v, leido);
    }
  }
  escribir(reg, antes);
  if (errores == 0) Serial.println("Comunicacion OK: lo que se escribe se lee igual (4 de 4).");
  else Serial.printf("FALLA de comunicacion: %d de 4 pruebas mal. Revisa cables, largo del bus y pull-ups.\n", errores);
}

bool iniciarPCA() {
  if (!existe(DIR_PCA)) { Serial.println("El PCA9685 (0x40) no responde."); return false; }
  prescale = (uint8_t)(OSC_HZ / (4096.0f * FREC_HZ) + 0.5f) - 1;
  bool ok = escribir(MODE1, 0x10);          // dormir para poder cambiar la frecuencia
  ok &= escribir(PRESCALE, prescale);
  ok &= escribir(MODE1, 0x20);              // despertar, auto incremento, sin All Call
  delay(2);
  ok &= escribir(MODE1, 0xA0);              // restart
  ok &= escribir(MODE2, 0x04);              // totem pole
  ok &= apagarTodo();
  Serial.printf("PCA9685 configurado: prescale %u, %.1f Hz, periodo %.0f us, salidas apagadas. %s\n",
                prescale, OSC_HZ / (4096.0f * (prescale + 1)), periodoUs(), ok ? "" : "(HUBO ERRORES I2C)");
  return ok;
}

void enviarPulso(float us) {
  if (us < 400 || us > 2600) { Serial.println("Usa un pulso entre 500 y 2500 us."); return; }
  fijar(canal, (us - 500) / 2000.0f * 180.0f);
  if (pulsoUs(canal, us)) Serial.printf("Canal %d: %.0f us (~%+.0f grados)\n", canal, us, (us - 500) / 2000.0f * 180.0f - 90);
  else Serial.println("Error I2C al escribir el canal.");
}

void barrido() {
  Serial.printf("Barrido del canal %d (-90 -> 90 -> -90). Envia cualquier tecla para cortar.\n", canal);
  for (int pasada = 0; pasada < 2; pasada++) {
    for (int k = 0; k <= 180; k += 2) {
      const int g = pasada == 0 ? k : 180 - k;
      pulsoUs(canal, gradosAUs(g));
      delay(30);
      if (Serial.available()) { while (Serial.available()) Serial.read(); Serial.println("Barrido cortado."); return; }
    }
  }
  fijar(canal, 0);
  Serial.println("Barrido terminado.");
}

void probarCanales() {
  Serial.println("Probando canales 0 a 15: cada uno al centro (0) por 1 s y luego se apaga.");
  Serial.println("Anota cual servo se mueve con cada canal.");
  for (int ch = 0; ch < 16; ch++) {
    Serial.printf("  Canal %2d ... ", ch);
    pulsoUs(ch, gradosAUs(90));
    delay(1000);
    apagar(ch); fijar(ch, 90);
    Serial.println("listo");
    if (Serial.available()) { while (Serial.available()) Serial.read(); Serial.println("Prueba cortada."); break; }
  }
}

// a = angulo con signo (0 = centro). Servo = 90 + a; en espejo el segundo va a 90 - a
void moverPar(const char *nombre, int c1, int c2, float a, bool espejo) {
  if (a < -90 || a > 90) { Serial.println("Usa un angulo entre -90 y 90 (0 = centro)."); return; }
  const float a2 = espejo ? 0.0f - a : a;   // 0 - a evita imprimir -0
  irSuave(c1, 90 + a);
  irSuave(c2, 90 + a2);
  Serial.printf("Par %s: canal %d a %+.0f, canal %d a %+.0f grados (%s)\n", nombre, c1, a, c2, a2, espejo ? "espejo" : "igual");
}

void medirPulso() {
  Serial.printf("Midiendo en GPIO %d (une la senal S del canal %d a ese pin)...\n", PIN_MEDIR, canal);
  float suma = 0; int n = 0;
  for (int k = 0; k < 10; k++) {
    const unsigned long t = pulseIn(PIN_MEDIR, HIGH, 50000);
    if (t > 0) { suma += t; n++; }
  }
  if (n == 0) {
    Serial.println("  No llega ningun pulso. Revisa:");
    Serial.println("  - que el canal tenga un pulso activo (ej: c0 y luego 1500)");
    Serial.println("  - el pin OE del PCA9685: debe ir a GND (si queda en alto, apaga todas las salidas)");
    Serial.println("  - el cable entre S y el GPIO 4");
    return;
  }
  uint8_t on[4];
  Wire.beginTransmission(DIR_PCA); Wire.write(LED0 + 4 * canal); Wire.endTransmission(false);
  Wire.requestFrom((int)DIR_PCA, 4);
  for (int k = 0; k < 4; k++) on[k] = Wire.read();
  const uint16_t ticks = (on[2] | (on[3] << 8)) & 0x0FFF;
  const float esperado = ticks * periodoUs() / 4096.0f, medido = suma / n;
  Serial.printf("  Pulso medido: %.0f us   esperado: %.0f us   (%d de 10 lecturas)\n", medido, esperado, n);
  if (esperado > 0) {
    const float oscReal = OSC_HZ * esperado / medido;
    Serial.printf("  Oscilador real aproximado: %.1f MHz", oscReal / 1e6f);
    Serial.println(fabs(medido - esperado) < 30 ? "  (OK)" : "  <- ajusta setOscillatorFrequency en main.cpp");
  }
}

void revisarTCA() {
  if (!existe(DIR_TCA)) { Serial.println("No responde nada en 0x70 (TCA9548A)."); return; }
  Serial.println("Canales del TCA9548A con un AS5600 (0x36):");
  for (int ch = 0; ch < 8; ch++) {
    Wire.beginTransmission(DIR_TCA); Wire.write(1 << ch); Wire.endTransmission();
    if (existe(DIR_AS5600)) Serial.printf("  canal %d: AS5600 encontrado\n", ch);
  }
  Wire.beginTransmission(DIR_TCA); Wire.write(0); Wire.endTransmission();
  Serial.println("  (el programa principal usa canal 0 = arriba y canal 1 = abajo)");
}

void ayuda() {
  Serial.println("Comandos:");
  Serial.println("  e escanear I2C   r registros   k prueba de comunicacion   i configurar a 50 Hz");
  Serial.println("  Angulos de -90 a 90, 0 = centro");
  Serial.println("  c<n> elegir canal   <us> pulso (ej 1500)   g<grados> angulo (g0, g-45)   b barrido");
  Serial.println("  p probar 16 canales   pa<g> / pb<g> par en espejo   qa<g> / qb<g> par igual   x apagar");
  Serial.println("  v<g/s> velocidad (ej v20)   m medir pulso en GPIO 4   t canales del TCA9548A   h ayuda");
}

// ---------------------------------------------------------------- comandos
void procesar(String s) {
  s.trim();
  if (s.length() == 0) return;
  const char c = tolower(s.charAt(0));
  if (isDigit(c)) { enviarPulso(s.toFloat()); return; }
  if (c == 'c' && s.length() > 1) {
    const int ch = s.substring(1).toInt();
    if (ch >= 0 && ch <= 15) { canal = ch; Serial.printf("Canal elegido: %d\n", canal); }
    else Serial.println("El canal va de 0 a 15.");
    return;
  }
  if (c == 'g' && s.length() > 1) {
    const float a = s.substring(1).toFloat();
    if (a >= -90 && a <= 90) { Serial.printf("Canal %d hacia %+.0f grados a %.0f grados/s\n", canal, a, velocidad); irSuave(canal, 90 + a); }
    else Serial.println("Usa un angulo entre -90 y 90 (0 = centro).");
    return;
  }
  if (c == 'v' && s.length() > 1) {
    const float v = s.substring(1).toFloat();
    if (v >= 2 && v <= 180) { velocidad = v; Serial.printf("Velocidad: %.0f grados/s\n", velocidad); }
    else Serial.println("Usa una velocidad entre 2 y 180 (ej: v20).");
    return;
  }
  if ((c == 'p' || c == 'q') && s.length() > 2) {   // p = espejo, q = igual (sin espejo)
    const char par = tolower(s.charAt(1));
    const float g = s.substring(2).toFloat();
    const bool espejo = (c == 'p');
    if (par == 'a') moverPar("arriba", 12, 13, g, espejo);
    else if (par == 'b') moverPar("abajo", 8, 9, g, espejo);
    else Serial.println("Usa pa<g>, pb<g> (espejo) o qa<g>, qb<g> (igual).");
    return;
  }
  if (s.length() == 1) {
    switch (c) {
      case 'e': escanear(); break;
      case 'r': explicarRegistros(); break;
      case 'k': pruebaComunicacion(); break;
      case 'i': iniciarPCA(); break;
      case 'b': barrido(); break;
      case 'p': probarCanales(); break;
      case 'x': apagarTodo(); for (int k = 0; k < 16; k++) detener(k); Serial.println("Todas las salidas apagadas (servos sin fuerza)."); break;
      case 'm': medirPulso(); break;
      case 't': revisarTCA(); break;
      case 'h': ayuda(); break;
      default: Serial.println("Comando no reconocido (h = ayuda)."); break;
    }
    return;
  }
  Serial.println("Comando no reconocido (h = ayuda).");
}

String linea;

void setup() {
  Serial.begin(115200);
  delay(500);
  pinMode(PIN_MEDIR, INPUT);
  memoria.begin("diagpca", false);
  for (int k = 0; k < 16; k++) {
    char clave[5]; snprintf(clave, sizeof clave, "c%d", k);
    actual[k] = memoria.getDouble(clave, NAN); objetivo[k] = NAN; velCanal[k] = 0;
  }
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);       // lento a proposito para el diagnostico
  Serial.println();
  Serial.println("=== Diagnostico PCA9685 ===");
  escanear();
  if (existe(DIR_PCA)) {
    Serial.println("--- Registros al encender ---");
    explicarRegistros();
    Serial.println("--- Comunicacion ---");
    pruebaComunicacion();
    Serial.println("--- Configuracion ---");
    iniciarPCA();
    Serial.println("--- Despues de configurar (0x70 solo debe aparecer si esta el TCA9548A) ---");
    escanear();
  }
  Serial.print("Posiciones guardadas (arrancan suave):");
  int guardadas = 0;
  for (int k = 0; k < 16; k++) if (!isnan(actual[k])) { Serial.printf(" c%d=%+.0f", k, actual[k] - 90); guardadas++; }
  Serial.println(guardadas ? "" : " ninguna todavia");
  ayuda();
  Serial.println("Siguiente paso: pa0 y pb0 (los dos pares al centro), luego pa30 y pa-30.");
}

void loop() {
  while (Serial.available()) {
    const char ch = Serial.read();
    if (ch == '\n' || ch == '\r') { procesar(linea); linea = ""; }
    else linea += ch;
  }
  tareasSuaves();
}