#ifndef PINS_H
#define PINS_H

#include <Arduino.h>

// ============================================================
//  Pines I2C del ESP32 para comunicar con el PCA9685
// ============================================================

#define PIN_SDA   21   // GPIO21 - linea de datos I2C
#define PIN_SCL   22   // GPIO22 - linea de reloj I2C

#define I2C_FREQ  100000  // 100 kHz (estandar, estable para PCA9685)

#endif
