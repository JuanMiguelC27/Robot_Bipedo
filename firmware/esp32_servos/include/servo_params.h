#ifndef SERVO_PARAMS_H
#define SERVO_PARAMS_H

#include <cstdint>

// ============================================================
//  Parámetros de los servos
//
//  Frecuencia PWM: 50 Hz (periodo 20 ms)
// ============================================================

#define PWM_FREQ_HZ      50
#define PCA9685_TICK_US  (1000000.0 / (PWM_FREQ_HZ * 4096.0))

#define OSC_FREQ_HZ      26492928.0f  // Oscilador PCA9685 (nominal)

// ============================================================
//  PRUEBA CON SG90 EN EL CANAL 0
//
//  1 -> el servo del canal 0 (Knee) es un SG90 (0-180°).
//  0 -> el canal 0 es el RDS51150 real (0-270°).
// ============================================================
#define SERVO_CANAL0_SG90  1

// ============================================================
//  Estructura de parámetros por servo
//
//  Los ángulos angleHome/angleMin/angleMax están en el MISMO
//  sistema que manda teleop_node por /servo_commands, es decir
//  ángulo cinemático + offset del teleop (0, 135, 135).
//
//  El ángulo físico del servo es:  fisico = comando - servoOffset
//  y el pulso:                     us = us0 + fisico * usPerDeg
// ============================================================
struct ServoParams {
    uint8_t channel;     // Canal PCA9685 (0-15)
    float angleHome;     // Ángulo de reposo (grados, sistema del comando)
    float angleMin;      // Límite inferior seguro (grados, sistema del comando)
    float angleMax;      // Límite superior seguro (grados, sistema del comando)
    float servoOffset;   // comando - servoOffset = ángulo físico del servo
    float us0;           // pulso en 0° físicos [us]
    float usPerDeg;      // us por grado físico
    const char* name;    // Nombre descriptivo
};

#define NUM_SERVOS 3

// RDS51150 (270°): 500-2500 us -> 0-270° (nominal, pendiente de calibrar)
#define RDS_US0        500.0f
#define RDS_US_PER_DEG 7.4074f

// SG90 (180°): 500-2400 us -> 0-180° (nominal)
#define SG90_US0        500.0f
#define SG90_US_PER_DEG 10.5556f

// ============================================================
//  CABLEADO REAL
//
//    junta lógica 0 (Hip Roll)  -> canal 2
//    junta lógica 1 (Hip Pitch) -> canal 1
//    junta lógica 2 (Knee)      -> canal 0
//
//  data[i] de /servo_commands es la junta lógica i.
// ============================================================

static const ServoParams SERVO_PARAMS[NUM_SERVOS] = {
    // channel, home, min, max, offset, us0, usPerDeg, name
    { 2, 180.0f, 165.0f, 270.0f, 0.0f, RDS_US0, RDS_US_PER_DEG, "Hip Roll"  },  // Junta 0 -> PWM2
    { 1, 135.0f,  45.0f, 225.0f, 0.0f, RDS_US0, RDS_US_PER_DEG, "Hip Pitch" },  // Junta 1 -> PWM1
#if SERVO_CANAL0_SG90
    // SG90: el teleop manda q3 + 135, así que q3 = 0° (135) queda
    // en el centro del SG90 (90° físicos). Rango q3 [-85°, +85°]
    // -> 5°..175° físicos (se deja margen a los topes del SG90).
    { 0, 135.0f,  50.0f, 220.0f, 45.0f, SG90_US0, SG90_US_PER_DEG, "Knee (SG90)" },  // Junta 2 -> PWM0
#else
    { 0, 135.0f,  45.0f, 225.0f, 0.0f, RDS_US0, RDS_US_PER_DEG, "Knee"      },  // Junta 2 -> PWM0
#endif
};

#endif
