#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include <math.h>

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/float32_multi_array.h>

#include "pins.h"
#include "servo_params.h"
#include "helpers.h"

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only available for Arduino framework with serial transport.
#endif

// ============================================================
// Robot Bipedo - Firmware ESP32 con micro-ROS
// Controla los servos via modulo PCA9685 (16 canales PWM, I2C).
//
// Conexiones:
//   ESP32 GND        -> PCA9685 GND
//   ESP32 3.3V       -> PCA9685 VCC  (logica)
//   ESP32 D21 (SDA)  -> PCA9685 SDA
//   ESP32 D22 (SCL)  -> PCA9685 SCL
//   ESP32 D4  (OE)   -> PCA9685 OE   (activo-bajo)
//   Fuente 5V        -> PCA9685 V+   (potencia de los servos)
//
//   Junta 0 -> PCA9685 PWM2 (Hip Roll)
//   Junta 1 -> PCA9685 PWM1 (Hip Pitch)
//   Junta 2 -> PCA9685 PWM0 (Knee)  <- SG90 de prueba si
//                                      SERVO_CANAL0_SG90 = 1
//
// Suscripcion:  /servo_commands  (std_msgs/Float32MultiArray)
//   data[0] -> angulo junta 0 (Hip Roll) en grados
//   data[1] -> angulo junta 1 (Hip Pitch) en grados
//   data[2] -> angulo junta 2 (Knee) en grados
//   (mismo sistema que manda teleop_node: q + offset 0/135/135)
//
// Publicacion:  /servo_states  (std_msgs/Float32MultiArray)
//   Posicion actual (grados, mismo sistema) de cada servo.
//
// Conexion con el agente:
//   El firmware espera al micro-ROS agent y, si el agente se
//   cae o se reinicia, se vuelve a conectar solo. Los servos
//   mantienen su ultima posicion mientras tanto.
//   LED: parpadeo lento = esperando agente, fijo = conectado.
// ============================================================

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { return false; } }
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
uint16_t last_ticks[NUM_SERVOS];

const float STEP_DEG = 0.3f;                 // grados por paso
const unsigned long STEP_PERIOD_MS = 10;     // 0.3° / 10 ms = 30 °/s
unsigned long last_step_time = 0;

// Estado de la conexion con el agente
enum AgentState {
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
};
AgentState agent_state = WAITING_AGENT;

// ============================================================
// Escribe un servo en el PCA9685 (solo si cambio el pulso)
// ============================================================
void writeServo(uint8_t servoId, float angleDeg) {
    uint16_t ticks = usToTicks(angleToUs(servoId, angleDeg));
    if (ticks != last_ticks[servoId]) {
        pwm.setPWM(SERVO_PARAMS[servoId].channel, 0, ticks);
        last_ticks[servoId] = ticks;
    }
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
}

// ============================================================
// Mueve los servos un paso hacia sus objetivos (sin saltos bruscos)
// ============================================================
void update_servos() {
    for (int i = 0; i < NUM_SERVOS; i++) {
        float diff = target_angle[i] - current_angle[i];

        if (fabsf(diff) > STEP_DEG) {
            current_angle[i] += (diff > 0.0f) ? STEP_DEG : -STEP_DEG;
        } else {
            current_angle[i] = target_angle[i];
        }

        writeServo(i, current_angle[i]);
        state_data[i] = current_angle[i];
    }
}

// ============================================================
// Timer: publica el estado de los servos
// ============================================================
void state_timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
    (void)timer;
    (void)last_call_time;
    RCSOFTCHECK(rcl_publish(&publisher, &state_msg, NULL));
}

// ============================================================
// Crea nodo, pub/sub, timer y executor (al conectar el agente)
// ============================================================
bool create_entities() {
    allocator = rcl_get_default_allocator();

    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "esp32_servo_controller", "", &support));

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

    const unsigned int timer_timeout = 500;
    RCCHECK(rclc_timer_init_default(
        &timer, &support, RCL_MS_TO_NS(timer_timeout), state_timer_callback));

    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_subscription(
        &executor, &subscription, &cmd_msg, &cmd_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    return true;
}

// ============================================================
// Libera todo (cuando se pierde el agente)
// ============================================================
void destroy_entities() {
    rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    RCSOFTCHECK(rcl_publisher_fini(&publisher, &node));
    RCSOFTCHECK(rcl_subscription_fini(&subscription, &node));
    RCSOFTCHECK(rcl_timer_fini(&timer));
    RCSOFTCHECK(rclc_executor_fini(&executor));
    RCSOFTCHECK(rcl_node_fini(&node));
    RCSOFTCHECK(rclc_support_fini(&support));
}

// ============================================================
// Setup
// ============================================================
void setup() {
    // Los mensajes de texto van ANTES de activar el transporte
    // micro-ROS: despues el puerto serie es solo del agente.
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
        last_ticks[i] = 0xFFFF;  // fuerza la primera escritura

        writeServo(i, SERVO_PARAMS[i].angleHome);
        delay(200);  // Espera entre servos para evitar picos
    }

    Serial.println("[ESP32] PCA9685 listo. Esperando micro-ROS agent...");
    Serial.flush();

    // Configurar micro-ROS transporte serial (UART0 -> USB)
    set_microros_serial_transports(Serial);
}

// ============================================================
// Loop
// ============================================================
void loop() {
    // Los servos se actualizan siempre, haya o no agente
    if (millis() - last_step_time >= STEP_PERIOD_MS) {
        last_step_time = millis();
        update_servos();
    }

    switch (agent_state) {
        case WAITING_AGENT: {
            static unsigned long last_ping = 0;
            if (millis() - last_ping >= 500) {
                last_ping = millis();
                digitalWrite(LED_PIN, !digitalRead(LED_PIN));  // parpadeo lento
                if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
                    agent_state = AGENT_AVAILABLE;
                }
            }
            break;
        }

        case AGENT_AVAILABLE:
            if (create_entities()) {
                agent_state = AGENT_CONNECTED;
                digitalWrite(LED_PIN, HIGH);
            } else {
                destroy_entities();
                agent_state = WAITING_AGENT;
            }
            break;

        case AGENT_CONNECTED: {
            static unsigned long last_check = 0;
            if (millis() - last_check >= 1000) {
                last_check = millis();
                if (rmw_uros_ping_agent(100, 3) != RMW_RET_OK) {
                    agent_state = AGENT_DISCONNECTED;
                    break;
                }
            }
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
            break;
        }

        case AGENT_DISCONNECTED:
            destroy_entities();
            digitalWrite(LED_PIN, LOW);
            agent_state = WAITING_AGENT;
            break;
    }
}
