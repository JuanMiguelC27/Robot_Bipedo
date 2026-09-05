#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "pins.h"
#include "servo_params.h"
#include "helpers.h"

// ============================================================
//  ServoController
//
//  Clase que gestiona el PCA9685 (I2C) para controlar
//  hasta 16 servos PWM. En esta pata se usan 3 canales.
//
//  Resolucion: 12 bits (0-4095)
//  Frecuencia PWM: 50 Hz (periodo 20 ms), configurada con
//  setOscillatorFrequency + setPWMFreq
// ============================================================

class ServoController {
public:
    ServoController();

    // Inicializa I2C y el PCA9685
    bool begin();

    // Mueve el servo indicado a un angulo en grados
    // servoId: 0..2
    // angleDeg: angulo en grados (se clamp a limites del servo)
    // retorna el PWM en us que se envio
    float setServoAngle(uint8_t servoId, float angleDeg);

    // Mueve un servo directamente con un valor PWM en us
    void setServoPWM(uint8_t servoId, float pwmUs);

    // Apaga todos los servos (libera los canales)
    void disableAll();

    // Lee el PWM actual configurado en un canal (en us)
    float getCurrentPWM(uint8_t servoId);

private:
    Adafruit_PWMServoDriver _pca;

    // Valores actuales en ticks (0-4095) de cada canal
    uint16_t _currentTicks[NUM_SERVOS];

    // Oscillator freq del PCA9685: 25 MHz (frecuencia del
    // oscilador interno del chip, valor usado por Adafruit).
    static constexpr float OSC_FREQ_HZ = 25000000.0f;
};

#endif
