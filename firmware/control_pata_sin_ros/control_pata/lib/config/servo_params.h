#ifndef SERVO_PARAMS_H
#define SERVO_PARAMS_H

#include <cstdint>

// ============================================================
//  Parametros de cada servo RDS51150
//
//  Rango PWM total: 270 grados (marco fisico 0-270)
//    500 us  ->   0 grados
//    1500 us -> 135 grados (centro mecanico / neutro)
//    2500 us -> 270 grados
//
//  Frecuencia PWM: 50 Hz (periodo 20 ms)
//  1 tick del PCA9685 a 50 Hz = 1000000/(50*4096) = 4.8828 us
// ============================================================

#define PWM_FREQ_HZ      50

// Tick del PCA9685 derivado de la frecuencia real
#define PCA9685_TICK_US  (1000000.0 / (PWM_FREQ_HZ * 4096.0))

// Pulso minimo y maximo fisicos del servo (us)
#define PWM_MIN_US   500.0f   // angulo 0 grados
#define PWM_NEUTRO_US 1500.0f // angulo 135 grados (neutro)
#define PWM_MAX_US   2500.0f  // angulo 270 grados

// Conversion lineal unica angulo (0-270, marco fisico) -> us
//   us = PWM_MIN_US + grados * US_PER_DEG
#define US_PER_DEG    ((PWM_MAX_US - PWM_MIN_US) / 270.0f)  // 7.4074 us/grado
#define ANGLE_TO_US(d) (PWM_MIN_US + (d) * US_PER_DEG)

// ============================================================
//  Estructura de parametros por servo
//
//  Solo guarda angulos: los limites de seguridad viven en
//  angleMin/angleMax y el PWM se deriva siempre con ANGLE_TO_US()
// ============================================================

struct ServoParams {
    uint8_t channel;       // Canal PCA9685 (0-15)
    float   angleHome;     // Angulo de reposo (grados)
    float   angleMin;      // Limite inferior de operacion (grados)
    float   angleMax;      // Limite superior de operacion (grados)
    const char* name;      // Nombre descriptivo del servo
};

#define NUM_SERVOS 3

// ============================================================
//  Definicion de los 3 servos de la pata
//
//  Servo 0 (Hip Roll):  180 a 200 grados (home en 180, limite inferior)
//  Servo 1 (Hip Pitch): 135 a 250 grados (home en 135, limite inferior)
//  Servo 2 (Knee):      135 a 250 grados (home en 135, limite inferior)
// ============================================================

static const ServoParams SERVO_PARAMS[NUM_SERVOS] = {
    // channel, angleHome, angleMin, angleMax, name
    { 0, 135.0f, 50.0f, 200.0f, "Hip Roll"  },
    { 1, 135.0f, 20.0f, 250.0f, "Hip Pitch" },
    { 2, 1.0f, 20.0f, 250.0f, "Knee"      },
};

#endif
