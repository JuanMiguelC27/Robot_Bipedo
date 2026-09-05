#include "servo_controller.h"

// ============================================================
//  Constructor
// ============================================================

ServoController::ServoController()
    : _pca(0x40)  // Direccion I2C del PCA9685
{
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
        _currentTicks[i] = 0;
    }
}

// ============================================================
//  begin() - Inicializa I2C y configura el PCA9685
// ============================================================

bool ServoController::begin() {
    // Iniciar I2C con los pines definidos
    Wire.begin(PIN_SDA, PIN_SCL, I2C_FREQ);

    // Iniciar el PCA9685
    if (!_pca.begin()) {
        return false;  // PCA9685 no encontrado en I2C
    }

    // Configurar la frecuencia del oscilador interno
    _pca.setOscillatorFrequency(OSC_FREQ_HZ);

    // Configurar PWM a 50 Hz (estandar para servos)
    _pca.setPWMFreq(PWM_FREQ_HZ);

    // Pequena espera para que el PCA9685 aplique la config
    delay(10);

    // Mover todos los servos al reposo al iniciar.
    // Se escalona con 200 ms entre servos para no disparar
    // el pico simultaneo de stall current (8 A por servo),
    // que podria resetear el ESP32 o corromper el bus I2C.
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
        setServoAngle(i, SERVO_PARAMS[i].angleHome);
        delay(200);
    }

    return true;
}

// ============================================================
//  setServoAngle() - Mueve un servo a un angulo en grados
// ============================================================

float ServoController::setServoAngle(uint8_t servoId, float angleDeg) {
    if (servoId >= NUM_SERVOS) return 0.0f;

    // Convertir angulo a PWM (us), ya clamp internamente
    float pwmUs = convertAngleToPWM(angleDeg, servoId);

    // Aplicar al PCA9685
    setServoPWM(servoId, pwmUs);

    return pwmUs;
}

// ============================================================
//  setServoPWM() - Mueve un canal con un PWM en microsegundos
// ============================================================

void ServoController::setServoPWM(uint8_t servoId, float pwmUs) {
    if (servoId >= NUM_SERVOS) return;

    const ServoParams& p = SERVO_PARAMS[servoId];

    // Clamp al rango PWM derivado de los angulos de este servo
    // (defensa en profundidad, el angulo ya fue clamp en
    // convertAngleToPWM)
    float pwmMin = ANGLE_TO_US(p.angleMin);
    float pwmMax = ANGLE_TO_US(p.angleMax);
    if (pwmUs < pwmMin) pwmUs = pwmMin;
    if (pwmUs > pwmMax) pwmUs = pwmMax;

    // Convertir microsegundos a ticks
    uint16_t ticks = pwmToTicks(pwmUs);

    // Enviar al canal correspondiente del PCA9685
    // pin 0 = on-ticks, pin = ticks = off-ticks
    _pca.setPWM(p.channel, 0, ticks);

    // Guardar valor actual
    _currentTicks[servoId] = ticks;
}

// ============================================================
//  disableAll() - Apaga todos los canales de servo
// ============================================================

void ServoController::disableAll() {
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
        // setPWM(ch, 0, 4096) activa el bit full-OFF del PCA9685,
        // apagando el canal de forma inequivoca (0,0 dejaria un
        // estado ambiguo de ON/OFF en el mismo valor).
        _pca.setPWM(SERVO_PARAMS[i].channel, 0, 4096);
        _currentTicks[i] = 0;
    }
}

// ============================================================
//  getCurrentPWM() - Retorna el PWM actual de un canal en us
// ============================================================

float ServoController::getCurrentPWM(uint8_t servoId) {
    if (servoId >= NUM_SERVOS) return 0.0f;
    return static_cast<float>(_currentTicks[servoId]) * PCA9685_TICK_US;
}
