#ifndef HELPERS_H
#define HELPERS_H

#include "servo_params.h"

// ============================================================
//  clampValue() - Limita un valor entre min y max
// ============================================================
template <typename T>
T clampValue(T value, T minVal, T maxVal) {
    if (value < minVal) return minVal;
    if (value > maxVal) return maxVal;
    return value;
}

// ============================================================
//  angleToUs() - Convierte ángulo (grados) a microsegundos PWM
//
//  Usa los valores nominales (o calibrados cuando los tengas)
//  para el servo específico.
// ============================================================
inline float angleToUs(uint8_t servoId, float angleDeg) {
    if (servoId >= NUM_SERVOS) return 1500.0f;  // Neutro por defecto
    
    const ServoParams& p = SERVO_PARAMS[servoId];
    
    // Clamp al rango seguro del servo
    float angle = clampValue(angleDeg, p.angleMin, p.angleMax);
    
    // Conversión lineal: us = US0 + grados * US_PER_DEG
    return US0 + angle * US_PER_DEG;
}

// ============================================================
//  usToTicks() - Convierte microsegundos a ticks del PCA9685
//
//  El PCA9685 tiene resolución de 12 bits (0-4095)
//  a la frecuencia configurada (50 Hz = 20ms periodo)
// ============================================================
inline uint16_t usToTicks(float pwmUs) {
    float ticks = pwmUs / PCA9685_TICK_US;
    
    // Clamp a rango 12-bit
    if (ticks < 0.0f) ticks = 0.0f;
    if (ticks > 4095.0f) ticks = 4095.0f;
    
    // Redondear en vez de truncar
    return static_cast<uint16_t>(ticks + 0.5f);
}

// ============================================================
//  ticksToUs() - Convierte ticks del PCA9685 a microsegundos
//  (Útil para debugging)
// ============================================================
inline float ticksToUs(uint16_t ticks) {
    return static_cast<float>(ticks) * PCA9685_TICK_US;
}

#endif