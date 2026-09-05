#ifndef SERVO_PARAMS_H
#define SERVO_PARAMS_H

#include <cstdint>

// ============================================================
//  Parámetros de los servos RDS51150 (270 grados)
//  VALORES NOMINALES - Pendientes de calibrar
//
//  Rango PWM: 500-2500 us para 0-270 grados
//  Frecuencia: 50 Hz
// ============================================================

#define PWM_FREQ_HZ      50
#define PCA9685_TICK_US  (1000000.0 / (PWM_FREQ_HZ * 4096.0))

// ============================================================
//  Valores nominales del servo RDS51150
//  (CALIBRAR DESPUÉS con el proyecto de calibración)
// ============================================================
#define OSC_FREQ_HZ      26492928.0f  // Oscilador PCA9685 (nominal)
#define US_PER_DEG       7.4074f      // us por grado (nominal)
#define US0              500.0f       // us en 0 grados (nominal)

// ============================================================
//  Estructura de parámetros por servo
// ============================================================
struct ServoParams {
    uint8_t channel;    // Canal PCA9685 (0-15)
    float angleHome;    // Ángulo de reposo (grados)
    float angleMin;     // Límite inferior seguro (grados)
    float angleMax;     // Límite superior seguro (grados)
    const char* name;   // Nombre descriptivo
};

#define NUM_SERVOS 3

// ============================================================
//  TUS SERVOS CON TU CABLEADO REAL
//
//  Cableado físico:
//    PWM0 -> junta 2 (Knee)
//    PWM1 -> junta 1 (Hip Pitch)
//    PWM2 -> junta 0 (Hip Roll)
//
//  Por tanto:
//    junta lógica 0 (Hip Roll)  -> canal 2
//    junta lógica 1 (Hip Pitch) -> canal 1
//    junta lógica 2 (Knee)      -> canal 0
// ============================================================

static const ServoParams SERVO_PARAMS[NUM_SERVOS] = {
    // channel, angleHome, angleMin, angleMax, name
    { 2, 180.0f, 50.0f, 200.0f, "Hip Roll"  },  // Junta 0 -> PWM2
    { 1, 135.0f, 20.0f, 250.0f, "Hip Pitch" },  // Junta 1 -> PWM1
    { 0, 135.0f, 20.0f, 250.0f, "Knee"      },  // Junta 2 -> PWM0
};

#endif