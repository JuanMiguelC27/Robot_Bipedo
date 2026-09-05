// ============================================================
//  FASE 1 - Test de servos con PCA9685 en ESP32 DEFINITIVO
//
//  Este sketch controla 3 servos RDS51150 conectados al
//  modulo PCA9685 via I2C. Permite ingregar angulos
//  desde la consola serial y opcionalmente una velocidad
//  (retardo entre movimientos) para observar el recorrido.
//
//  No incluye ROS ni logica de comunicacion avanzada.
//
//  Formato de entrada serial:
//    <hiproll> <hippitch> <knee>
//    Ejemplo: 125 135 135
//
//  Opcionalmente se puede especificar velocidad (ms):
//    <hiproll> <hippitch> <knee> <velocidad_ms>
//    Ejemplo: 125 135 135 20
// ============================================================

#include <Arduino.h>
#include "servo_controller.h"
#include "servo_params.h"

// Objeto global del controlador de servos
ServoController servoCtrl;

// Tiempo del loop principal (10 ms)
const unsigned long LOOP_INTERVAL_MS = 10;

// Velocidad de interpolacion (ms por paso)
// Valor por defecto: sin movimiento suave, instantaneo
uint16_t moveSpeedMs = 0;

// Angulos objetivo (grados)
float targetAngles[NUM_SERVOS] = { 0, 0, 0 };

// Angulos actuales (grados) - para interpolacion
float currentAngles[NUM_SERVOS] = { 0, 0, 0 };

// Timestamp del ultimo ciclo
unsigned long lastLoopTime = 0;

// ============================================================
//  setup()
// ============================================================

void setup() {
    Serial.begin(115200);
    // Evita que readStringUntil() bloquee hasta 1000 ms por
    // defecto si el monitor no envia '\n'; con 20 ms el ciclo
    // de 10 ms no se rompe.
    Serial.setTimeout(20);
    delay(500);

    Serial.println("========================================");
    Serial.println("  FASE 1 - Test Servos PCA9685 (ESP32)");
    Serial.println("========================================");
    Serial.println();

    // Inicializar el controlador de servos
    if (!servoCtrl.begin()) {
        Serial.println("[ERROR] PCA9685 no detectado en I2C!");
        Serial.println("  Verificar conexiones SDA/SCL y alimentacion.");
        while (true) {
            delay(1000);  // Detenerse si no hay modulo
        }
    }

    Serial.println("[OK] PCA9685 inicializado correctamente.");
    Serial.println();

    // Imprimir info de cada servo
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
        const ServoParams& p = SERVO_PARAMS[i];
        Serial.print("  Servo ");
        Serial.print(i);
        Serial.print(" [");
        Serial.print(p.name);
        Serial.print("]: canal ");
        Serial.print(p.channel);
        Serial.print(", rango ");
        Serial.print(p.angleMin, 0);
        Serial.print("-");
        Serial.print(p.angleMax, 0);
        Serial.println(" grados");
    }

    Serial.println();
    Serial.println("Formato de entrada:");
    Serial.println("  <hiproll> <hippitch> <knee>");
    Serial.println("  <hiproll> <hippitch> <knee> <velocidad_ms>");
    Serial.println();
    Serial.print("  Valores en grados (marco 0-270, rangos: hiproll=");
    Serial.print(SERVO_PARAMS[0].angleMin, 0);
    Serial.print("-");
    Serial.print(SERVO_PARAMS[0].angleMax, 0);
    Serial.print(", hippitch=");
    Serial.print(SERVO_PARAMS[1].angleMin, 0);
    Serial.print("-");
    Serial.print(SERVO_PARAMS[1].angleMax, 0);
    Serial.print(", knee=");
    Serial.print(SERVO_PARAMS[2].angleMin, 0);
    Serial.print("-");
    Serial.print(SERVO_PARAMS[2].angleMax, 0);
    Serial.println(" grados)");

    // Angulos iniciales en neutro del primer servo
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
        currentAngles[i] = (SERVO_PARAMS[i].angleHome);
        targetAngles[i] = currentAngles[i];
    }

    lastLoopTime = millis();
}

// ============================================================
//  parseSerialInput() - Lee y procesa comandos del monitor
//
//  Formato: <hiproll> <hippitch> <knee> [velocidad_ms]
//    angulos en grados (marco fisico 0-270)
//    velocidad_ms (opcional): retardo entre pasos de
//    interpolacion (0 = instantaneo)
//
//  Retorna true si se recibio un comando valido
// ============================================================

bool parseSerialInput() {
    if (!Serial.available()) return false;

    // Leer linea completa
    String line = Serial.readStringUntil('\n');
    line.trim();

    if (line.length() == 0) return false;

    // Parsear valores separados por espacio
    float hipRoll, hipPitch, knee;
    int speedMs = 0;
    int parsed = sscanf(line.c_str(), "%f %f %f %d",
                        &hipRoll, &hipPitch, &knee, &speedMs);

    if (parsed < 3) {
        Serial.println("[WARN] Formato invalido. Use: <hiproll> <hippitch> <knee>");
        return false;
    }

    // Validar rangos
    bool valid = true;

    if (hipRoll < SERVO_PARAMS[0].angleMin || hipRoll > SERVO_PARAMS[0].angleMax) {
        Serial.print("[WARN] Hip Roll fuera de rango (");
        Serial.print(SERVO_PARAMS[0].angleMin, 0);
        Serial.print("-");
        Serial.print(SERVO_PARAMS[0].angleMax, 0);
        Serial.println(" grados)");
        valid = false;
    }

    if (hipPitch < SERVO_PARAMS[1].angleMin || hipPitch > SERVO_PARAMS[1].angleMax) {
        Serial.print("[WARN] Hip Pitch fuera de rango (");
        Serial.print(SERVO_PARAMS[1].angleMin, 0);
        Serial.print("-");
        Serial.print(SERVO_PARAMS[1].angleMax, 0);
        Serial.println(" grados)");
        valid = false;
    }

    if (knee < SERVO_PARAMS[2].angleMin || knee > SERVO_PARAMS[2].angleMax) {
        Serial.print("[WARN] Knee fuera de rango (");
        Serial.print(SERVO_PARAMS[2].angleMin, 0);
        Serial.print("-");
        Serial.print(SERVO_PARAMS[2].angleMax, 0);
        Serial.println(" grados)");
        valid = false;
    }

    if (!valid) return false;

    // Aplicar angulos objetivo
    targetAngles[0] = hipRoll;
    targetAngles[1] = hipPitch;
    targetAngles[2] = knee;

    // Aplicar velocidad si se proporciono
    if (parsed >= 4 && speedMs >= 0) {
        moveSpeedMs = static_cast<uint16_t>(speedMs);
    }

    // Feedback
    Serial.print("[SET] Hip Roll=");
    Serial.print(hipRoll, 1);
    Serial.print("  Hip Pitch=");
    Serial.print(hipPitch, 1);
    Serial.print("  Knee=");
    Serial.print(knee, 1);
    Serial.print("  Vel=");
    Serial.print(moveSpeedMs);
    Serial.println("ms");

    return true;
}

// Tolerancia para comparar angulos (evita == en floats)
const float ANGLE_EPS = 0.01f;

// ============================================================
//  interpolateServos() - Movimiento suave hacia el target
//
//  Si moveSpeedMs == 0, movimiento instantaneo.
//  Si moveSpeedMs > 0, interpola linealmente con ese paso.
//
//  El temporizador se evalua UNA vez antes del bucle y solo
//  se actualiza cuando efectivamente se ejecuta un paso, para
//  que el intervalo entre pasos sea real (moveSpeedMs) y no la
//  duracion del loop (10 ms).
// ============================================================

void interpolateServos() {
    static unsigned long lastMoveTime = 0;
    unsigned long now = millis();

    if (moveSpeedMs == 0) {
        // Movimiento instantaneo
        for (uint8_t i = 0; i < NUM_SERVOS; i++) {
            if (fabs(currentAngles[i] - targetAngles[i]) < ANGLE_EPS) continue;
            currentAngles[i] = targetAngles[i];
            servoCtrl.setServoAngle(i, currentAngles[i]);
        }
        return;
    }

    // Movimiento suave: evaluar el temporizador una sola vez
    if (now - lastMoveTime < moveSpeedMs) return;
    lastMoveTime = now;

    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
        float diff = targetAngles[i] - currentAngles[i];
        // Ya llego al target: no reenviar por I2C
        if (fabs(diff) < ANGLE_EPS) continue;

        float step = 1.0f;  // 1 grado por paso

        if (diff > step) {
            currentAngles[i] += step;
        } else if (diff < -step) {
            currentAngles[i] -= step;
        } else {
            currentAngles[i] = targetAngles[i];
        }

        // Enviar al servo
        servoCtrl.setServoAngle(i, currentAngles[i]);
    }
}

// ============================================================
//  printStatus() - Imprime estado actual periodicamente
// ============================================================

void printStatus() {
    static unsigned long lastPrintTime = 0;
    unsigned long now = millis();

    // Imprimir cada 500 ms
    if (now - lastPrintTime < 500) return;
    lastPrintTime = now;

    Serial.print("[POS] ");
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
        Serial.print(SERVO_PARAMS[i].name);
        Serial.print("=");
        Serial.print(currentAngles[i], 1);
        Serial.print("deg  ");
    }
    Serial.println();
}

// ============================================================
//  loop() - Ciclo principal cada 10 ms
// ============================================================

void loop() {
    unsigned long now = millis();

    // Control de timing: ejecutar cada 10 ms
    if (now - lastLoopTime < LOOP_INTERVAL_MS) return;
    lastLoopTime = now;

    // 1. Leer comandos del serial
    parseSerialInput();

    // 2. Interpolar hacia angulos objetivo
    interpolateServos();

    // 3. Imprimir estado periodicamente
    printStatus();
}
