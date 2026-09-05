#ifndef PINS_H
#define PINS_H

#include <Arduino.h>

// ============================================================
//  Pines del ESP32 para el robot bípedo
// ============================================================

// I2C para PCA9685
#define PIN_SDA   21   // GPIO21 - línea de datos I2C
#define PIN_SCL   22   // GPIO22 - línea de reloj I2C
#define I2C_FREQ  100000  // 100 kHz (estándar para PCA9685)

// Control del PCA9685
#define PIN_OE    4    // GPIO4 - Output Enable (activo-bajo)

// LED indicador
#define LED_PIN   2    // LED onboard del ESP32

#endif