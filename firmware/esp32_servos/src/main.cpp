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
//   Junta 0 -> PCA9685 PWM0
//   Junta 1 -> PCA9685 PWM1
//   Junta 2 -> PCA9685 PWM2
//   Los servos (RDS51150) se alimentan con fuente 12V independiente.
//
// Suscripcion:  /servo_commands  (std_msgs/Float32MultiArray)
//   data[0] -> angulo junta 0 (PWM0) en grados FISICOS [0-270]
//   data[1] -> angulo junta 1 (PWM1) en grados FISICOS [0-270]
//   data[2] -> angulo junta 2 (PWM2) en grados FISICOS [0-270]
//   (135 = neutro = posicion recta de la pata)
//
// Publicacion:  /servo_states  (std_msgs/Float32MultiArray)
//   Posicion actual (grados) de cada servo.
// ============================================================

#define LED_PIN 2
#define PCA_SDA 21
#define PCA_SCL 22
#define PCA_OE  4
#define PCA_ADDR 0x40
#define NUM_SERVOS 3

// Canales del PCA9685 asignados a cada junta.
// CABLEADO FISICO REAL:
//   PWM0 -> junta 2
//   PWM1 -> junta 1
//   PWM2 -> junta 0
// Por tanto, el canal del PCA9685 para la junta logica i es:
//   junta 0 -> PWM2, junta 1 -> PWM1, junta 2 -> PWM0
const uint8_t servo_channels[NUM_SERVOS] = { 2, 1, 0 };

// Especificacion servo RDS51150 (270°):
//   pulse width 500-2500us
//   frequencia de operacion 50-330Hz (usamos 50Hz = periodo 20000us)
//
// ESCALA FISICA REAL del servo (segun datasheet):
//   500 us  ->   0 grados
//   1500 us -> 135 grados (neutro = posicion recta/alineada de la pata)
//   2500 us -> 270 grados
// Pendiente: (2500-500)/270 = 7.407 us/grado.
// /servo_commands transporta el angulo FISICO del servo en [0-270].
#define SERVO_PULSE_MIN_US  500
#define SERVO_PULSE_MAX_US  2500
#define SERVO_ANG_MIN       0.0f
#define SERVO_ANG_MAX       270.0f
#define SERVO_PWM_FREQ_HZ   50

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { error_loop(); } }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) {} }

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA_ADDR);

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

// Objetivos y posiciones actuales para transicion suave.
// Movemos los servos gradualmente para reducir picos de corriente que
// derrumban el voltaje.
float target_angle[NUM_SERVOS] = { 0.0f, 0.0f, 0.0f };
float current_angle[NUM_SERVOS] = { 0.0f, 0.0f, 0.0f };
unsigned long last_move_time = 0;

// ============================================================
// Convierte un angulo (grados) a el "tick" del PCA9685 (0-4095)
// para la frecuencia configurada.
// ============================================================
int angle_to_pca_tick(float angle_deg) {
  if (angle_deg < SERVO_ANG_MIN) angle_deg = SERVO_ANG_MIN;
  if (angle_deg > SERVO_ANG_MAX) angle_deg = SERVO_ANG_MAX;

  // Pendiente real del servo: 2000 us sobre 270 grados = 7.407 us/grado
  float ratio = angle_deg / (SERVO_ANG_MAX - SERVO_ANG_MIN); // 0..1
  float pulse_us = SERVO_PULSE_MIN_US +
                   ratio * (SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US);
    //float pulse_us = (500.0f + (angle_deg)*7.40740740741f);

  // ticks = pulso_us * 4096 / periodo_us
  float period_us = 1000000.0f / SERVO_PWM_FREQ_HZ;
  int tick = (int)roundf(pulse_us * 4096.0f / period_us);
  return tick;
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
    if (ang < SERVO_ANG_MIN) ang = SERVO_ANG_MIN;
    if (ang > SERVO_ANG_MAX) ang = SERVO_ANG_MAX;
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
      float step = (diff > 0.0f) ? 2.0f : -2.0f;
      current_angle[i] += step;
      if (current_angle[i] < SERVO_ANG_MIN) current_angle[i] = SERVO_ANG_MIN;
      if (current_angle[i] > SERVO_ANG_MAX) current_angle[i] = SERVO_ANG_MAX;
    } else {
      current_angle[i] = target_angle[i];
    }

    // Escribir al canal del PCA9685
    pwm.setPWM(servo_channels[i], 0, angle_to_pca_tick(current_angle[i]));
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
  Serial.println("\n[ESP32] Booting servo controller (PCA9685)...");

  pinMode(LED_PIN, OUTPUT);
  pinMode(PCA_OE, OUTPUT);
  // OE es activo-bajo: LOW = salidas del PCA9685 ACTIVAS
  digitalWrite(PCA_OE, LOW);

  // Inicializar I2C e PCA9685
  Wire.begin(PCA_SDA, PCA_SCL);
  pwm.begin();
  pwm.setOscillatorFrequency(30390682); // 27MHz tipico del PCA9685
  pwm.setPWMFreq(SERVO_PWM_FREQ_HZ);
  delay(10);

  // Posicion inicial segura para la estructura del robot.
  // La pata recta/alineada corresponde al offset de cada junta:
  //   junta 0 -> 180 grados fisicos
  //   junta 1 -> 135 grados fisicos
  //   junta 2 -> 135 grados fisicos
  const float home_angle[NUM_SERVOS] = { 180.0f, 135.0f, 135.0f };
  for (int i = 0; i < NUM_SERVOS; i++) {
    cmd_data[i] = 0.0f;
    target_angle[i] = home_angle[i];
    current_angle[i] = home_angle[i];
    state_data[i] = home_angle[i];
    pwm.setPWM(servo_channels[i], 0, angle_to_pca_tick(current_angle[i]));
  }

  Serial.println("[ESP32] PCA9685 initialized. Servos en posicion inicial");

  // Configurar micro-ROS transporte serial (UART0 -> USB)
  set_microros_serial_transports(Serial);
  Serial.println("[ESP32] micro-ROS serial transport set");

  allocator = rcl_get_default_allocator();

  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
    Serial.println("[ESP32] FAIL rclc_support_init");
    error_loop();
  }
  if (rclc_node_init_default(&node, "esp32_servo_controller", "", &support) != RCL_RET_OK) {
    Serial.println("[ESP32] FAIL rclc_node_init_default");
    error_loop();
  }
  Serial.println("[ESP32] micro-ROS node created");

  cmd_msg.data.data = cmd_data;
  cmd_msg.data.capacity = NUM_SERVOS;
  cmd_msg.data.size = 0;

  if (rclc_subscription_init_default(
      &subscription, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
      "servo_commands") != RCL_RET_OK) {
    Serial.println("[ESP32] FAIL subscription");
    error_loop();
  }

  state_msg.data.data = state_data;
  state_msg.data.capacity = NUM_SERVOS;
  state_msg.data.size = NUM_SERVOS;

  if (rclc_publisher_init_default(
      &publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
      "servo_states") != RCL_RET_OK) {
    Serial.println("[ESP32] FAIL publisher");
    error_loop();
  }
  Serial.println("[ESP32] pub/sub created");

  const unsigned int timer_timeout = 500;
  if (rclc_timer_init_default(
      &timer, &support, RCL_MS_TO_NS(timer_timeout), state_timer_callback) != RCL_RET_OK) {
    Serial.println("[ESP32] FAIL timer");
    error_loop();
  }

  if (rclc_executor_init(&executor, &support.context, 2, &allocator) != RCL_RET_OK) {
    Serial.println("[ESP32] FAIL executor");
    error_loop();
  }
  rclc_executor_add_subscription(
      &executor, &subscription, &cmd_msg, &cmd_callback, ON_NEW_DATA);
  rclc_executor_add_timer(&executor, &timer);

  digitalWrite(LED_PIN, LOW);
  Serial.println("[ESP32] Setup complete. Waiting for agent on /dev/ttyUSB0...");
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