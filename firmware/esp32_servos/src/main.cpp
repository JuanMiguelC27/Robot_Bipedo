#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include <math.h>

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32_multi_array.h>

#include "pins.h"
#include "servo_params.h"
#include "helpers.h"

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only available for Arduino framework with serial transport.
#endif

// ============================================================
// Robot Bipedo - Firmware ESP32 con micro-ROS
// Controla 3 servos RDS51150 via modulo PCA9685 (16 canales PWM, I2C).
//
// Conexiones:
//   ESP32 D21 (SDA)  -> PCA9685 SDA
//   ESP32 D22 (SCL)  -> PCA9685 SCL
//   ESP32 D4  (OE)   -> PCA9685 OE   (activo-bajo)
//   Junta 0 -> PCA9685 PWM2 (Hip Roll)
//   Junta 1 -> PCA9685 PWM1 (Hip Pitch)
//   Junta 2 -> PCA9685 PWM0 (Knee)
//   Los servos (RDS51150) se alimentan con fuente 12V independiente.
//
// Suscripcion:  /servo_commands  (std_msgs/Float32MultiArray)
//   data[0] -> angulo junta 0 (Hip Roll) en grados
//   data[1] -> angulo junta 1 (Hip Pitch) en grados
//   data[2] -> angulo junta 2 (Knee) en grados
//
// Publicacion:  /servo_states  (std_msgs/Float32MultiArray)
//   Posicion actual (grados) de cada servo.
// ============================================================

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { error_loop(); } }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) {} }

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// micro-ROS objetos
rclc_support_t support;
rcl_node_t node;
rcl_allocator_t allocator;
rcl_subscription_t subscription;
rcl_publisher_t publisher;
rclc_executor_t executor;
rcl_timer_t timer;

std_msgs__msg__Float32MultiArray cmd_msg;
std_msgs__msg__Float32MultiArray state_msg;

float cmd_data[NUM_SERVOS];
float state_data[NUM_SERVOS];

// Objetivos y posiciones actuales para transicion suave
float target_angle[NUM_SERVOS];
float current_angle[NUM_SERVOS];
unsigned long last_move_time = 0;

// Antibacklash
const bool ANTI_BACKLASH = true;
const float APPROACH_OFFSET_DEG = 3.0f;  // 3 grados de aproximación
const unsigned long SETTLE_MS = 100;      // Tiempo de asentamiento

// ============================================================
// Mueve un servo con antibacklash
// ============================================================
void moveServo(uint8_t servoId, float angleDeg) {
    const ServoParams& p = SERVO_PARAMS[servoId];
    
    if (ANTI_BACKLASH) {
        // Aproximar desde abajo para eliminar juego de engranajes
        float approachAngle = angleDeg - APPROACH_OFFSET_DEG;
        if (approachAngle < p.angleMin) approachAngle = p.angleMin;
        
        if (approachAngle < angleDeg - 0.5f) {
            float approachUs = angleToUs(servoId, approachAngle);
            pwm.setPWM(p.channel, 0, usToTicks(approachUs));
            delay(SETTLE_MS);
        }
    }
    
    // Mover al objetivo final
    float targetUs = angleToUs(servoId, angleDeg);
    pwm.setPWM(p.channel, 0, usToTicks(targetUs));
    delay(SETTLE_MS);
}

// ============================================================
// Callback: recibe angulos para los 3 servos (guarda objetivos)
// ============================================================
void cmd_callback(const void* msgin) {
    const std_msgs__msg__Float32MultiArray* msg =
        (const std_msgs__msg__Float32MultiArray*)msgin;

    if (msg->data.size < NUM_SERVOS) return;

    for (int i = 0; i < NUM_SERVOS; i++) {
        float ang = msg->data.data[i];
        // Clamp a límites específicos del servo
        ang = clampValue(ang, SERVO_PARAMS[i].angleMin, SERVO_PARAMS[i].angleMax);
        target_angle[i] = ang;
    }
    last_move_time = millis();
}

// ============================================================
// Mueve los servos un paso hacia sus objetivos (sin saltos bruscos)
// ============================================================
void update_servos() {
    for (int i = 0; i < NUM_SERVOS; i++) {
        float diff = target_angle[i] - current_angle[i];
        
        if (fabsf(diff) > 1.0f) {
            float step = (diff > 0.0f) ? 0.3f : -0.3f;
            current_angle[i] += step;
            current_angle[i] = clampValue(current_angle[i], 
                                          SERVO_PARAMS[i].angleMin, 
                                          SERVO_PARAMS[i].angleMax);
        } else {
            current_angle[i] = target_angle[i];
        }

        // Escribir al canal del PCA9685
        float us = angleToUs(i, current_angle[i]);
        pwm.setPWM(SERVO_PARAMS[i].channel, 0, usToTicks(us));
        state_data[i] = current_angle[i];
    }
}

// ============================================================
// Timer: publica el estado de los servos
// ============================================================
void state_timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
    (void)timer;
    (void)last_call_time;
    if (rcl_publisher_is_valid(&publisher)) {
        RCSOFTCHECK(rcl_publish(&publisher, &state_msg, NULL));
    }
}

// ============================================================
// Bucle de error
// ============================================================
void error_loop(void) {
    while (1) {
        digitalWrite(LED_PIN, HIGH);
        delay(150);
        digitalWrite(LED_PIN, LOW);
        delay(150);
    }
}

// ============================================================
// Setup
// ============================================================
void setup() {
    Serial.begin(115200);
    delay(300);
    Serial.println("\n[ESP32] Booting biped servo controller...");

    pinMode(LED_PIN, OUTPUT);
    pinMode(PIN_OE, OUTPUT);
    // OE es activo-bajo: LOW = salidas del PCA9685 ACTIVAS
    digitalWrite(PIN_OE, LOW);

    // Inicializar I2C y PCA9685
    Wire.begin(PIN_SDA, PIN_SCL, I2C_FREQ);
    pwm.begin();
    pwm.setOscillatorFrequency(OSC_FREQ_HZ);  // Nominal por ahora
    pwm.setPWMFreq(PWM_FREQ_HZ);
    delay(10);

    // Posicion inicial escalonada (evitar picos de corriente)
    Serial.println("[ESP32] Moviendo a posicion home...");
    for (int i = 0; i < NUM_SERVOS; i++) {
        target_angle[i] = SERVO_PARAMS[i].angleHome;
        current_angle[i] = SERVO_PARAMS[i].angleHome;
        state_data[i] = SERVO_PARAMS[i].angleHome;
        
        float us = angleToUs(i, SERVO_PARAMS[i].angleHome);
        pwm.setPWM(SERVO_PARAMS[i].channel, 0, usToTicks(us));
        delay(200);  // Espera entre servos para evitar picos
    }

    Serial.println("[ESP32] PCA9685 initialized");

    // Configurar micro-ROS transporte serial (UART0 -> USB)
    set_microros_serial_transports(Serial);
    Serial.println("[ESP32] micro-ROS serial transport set");

    allocator = rcl_get_default_allocator();

    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "esp32_servo_controller", "", &support));
    Serial.println("[ESP32] micro-ROS node created");

    cmd_msg.data.data = cmd_data;
    cmd_msg.data.capacity = NUM_SERVOS;
    cmd_msg.data.size = 0;

    RCCHECK(rclc_subscription_init_default(
        &subscription, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "servo_commands"));

    state_msg.data.data = state_data;
    state_msg.data.capacity = NUM_SERVOS;
    state_msg.data.size = NUM_SERVOS;

    RCCHECK(rclc_publisher_init_default(
        &publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "servo_states"));
    Serial.println("[ESP32] pub/sub created");

    const unsigned int timer_timeout = 500;
    RCCHECK(rclc_timer_init_default(
        &timer, &support, RCL_MS_TO_NS(timer_timeout), state_timer_callback));

    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_subscription(
        &executor, &subscription, &cmd_msg, &cmd_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    digitalWrite(LED_PIN, LOW);
    Serial.println("[ESP32] Setup complete. Waiting for agent...");
}

// ============================================================
// Loop
// ============================================================
void loop() {
    if (millis() - last_move_time < 5 || millis() - last_move_time > 20) {
        update_servos();
    }
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    delay(1);
}