#ifndef HELPERS_H
#define HELPERS_H

#include "servo_params.h"

// ============================================================
//  clampValue() - limita un valor entre min y max
// ============================================================

template <typename T>
T clampValue(T value, T minVal, T maxVal) {
    if (value < minVal) return minVal;
    if (value > maxVal) return maxVal;
    return value;
}

// ============================================================
//  convertAngleToPWM() - convierte un angulo (grados)
//  al valor de PWM en us para un servo determinado.
//
//  Usa la conversion unica ANGLE_TO_US() (marco 0-270),
//  por lo que todos los servos usan la misma escala.
//
//  El angulo se clampa a [angleMin, angleMax] antes de
//  convertir para nunca enviar un valor fuera de rango.
// ============================================================

inline float convertAngleToPWM(float angleDeg, uint8_t servoId) {
    // Validar indice
    if (servoId >= NUM_SERVOS) return PWM_NEUTRO_US;

    const ServoParams& p = SERVO_PARAMS[servoId];

    // Proteger contra rango degenerado (angleMin == angleMax)
    if (p.angleMax == p.angleMin) return ANGLE_TO_US(p.angleMin);

    // Clamp al rango operativo del servo
    float angle = clampValue(angleDeg, p.angleMin, p.angleMax);

    // Conversion lineal unica: angulo -> us
    return ANGLE_TO_US(angle);
}

// ============================================================
//  pwmToTicks() - convierte un PWM en microsegundos
//  al valor de ticks del registro del PCA9685.
//
//  tick = pwm_us / PCA9685_TICK_US
//  Se redondea (0.5) en vez de truncar para evitar un
//  sesgo sistematico (1 tick = 4.88 us = 0.66 grados).
// ============================================================

inline uint16_t pwmToTicks(float pwmUs) {
    float ticks = pwmUs / PCA9685_TICK_US;
    // Clamp a 12-bit (0-4095)
    if (ticks < 0.0f)   ticks = 0.0f;
    if (ticks > 4095.0f) ticks = 4095.0f;
    // Redondear en vez de truncar
    return static_cast<uint16_t>(ticks + 0.5f);
}

#endif
