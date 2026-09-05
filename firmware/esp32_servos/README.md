# Firmware ESP32 - Control de 3 Servos via micro-ROS

Controla 3 servos conectados a los GPIO **12, 13 y 14** de una **ESP32 DevKit V1**
usando **micro-ROS** con transporte **serial (USB)**.

## Arquitectura

```
+------------------+   /servo_commands    +--------------------------------+
|   ROS2 (host)    | -------------------> |  micro-ROS Agent (USB0 -> host) |
|  publica orden   |  Float32MultiArray   +--------------------------------+
+------------------+                                        |
                                                            | serial
                                                            v
+-------------------------------------------------------------+
|                      ESP32 (firmware)                       |
|  - Suscribe a /servo_commands                               |
|  - Controla servos en GPIO 12, 13, 14 (PWM 500-2500us)      |
|  - Publica estado en /servo_states                          |
+-------------------------------------------------------------+
```

## Contenido

| Archivo | Descripcion |
|---|---|
| `platformio.ini` | Configuracion de PlatformIO (board, micro-ROS, dependencias) |
| `src/main.cpp` | Firmware: suscripcion + control de servos + publicacion de estado |

## Temas

- **Suscripcion:** `/servo_commands` (`std_msgs/Float32MultiArray`)
  - `data[0]` -> angulo servo GPIO 12 (0-180 grados)
  - `data[1]` -> angulo servo GPIO 13 (0-180 grados)
  - `data[2]` -> angulo servo GPIO 14 (0-180 grados)
- **Publicacion:** `/servo_states` (`std_msgs/Float32MultiArray`)
  - Posicion actual (grados) de cada servo (cada 500 ms)

## Conexion de los servos

| Servo | Señal (GPIO) | Alimentacion |
|---|---|---|
| Servo 0 | GPIO 12 | 5V externo |
| Servo 1 | GPIO 13 | 5V externo |
| Servo 2 | GPIO 14 | 5V externo |
| GND (comun) | GND | GND de ESP32 + fuente externa |

> IMPORTANTE: Alimenta los servos desde una fuente externa de 5V (no desde el
> pin 5V de la ESP32 si son 3 servos a la vez). Conecta el GND de la fuente al
> GND de la ESP32.

## Pasos

### 1. Flashear firmware

Con la ESP32 conectada al puerto `/dev/ttyUSB0`:

```bash
cd firmware/esp32_servos
pio run -t upload --upload-port /dev/ttyUSB0
```

### 2. Levantar el micro-ROS Agent (serial)

En una terminal aparte, desactiva cualquier monitor serial y lanza:

```bash
source /opt/ros/humble/setup.bash
source ~/microros_ws/install/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -v6
```

Deberia verse algo como `Client connected` cuando la ESP32 se conecte.

### 3. Verificar nodo en ROS2

En otra terminal:

```bash
source /opt/ros/humble/setup.bash
ros2 node list            # deberia aparecer /esp32_servo_controller
ros2 topic list           # deberia estar /servo_commands y /servo_states
ros2 topic echo /servo_states
```

### 4. Mover los servos

```bash
# Movil todos los servos a 90 grados
ros2 topic pub --once /servo_commands std_msgs/msg/Float32MultiArray "{data: [90.0, 90.0, 90.0]}"

# Servo 0 a 45°, servo 1 a 90°, servo 2 a 135°
ros2 topic pub --once /servo_commands std_msgs/msg/Float32MultiArray "{data: [45.0, 90.0, 135.0]}"
```

## Tipos de mensaje

El firmware usa `std_msgs/Float32MultiArray` que ya viene incluido en la libreria
micro-ROS estandar, por lo que no requiere generar mensajes custom.

## Notas

- La compilacion de la stack micro-ROS tarda la primera vez (descarga y compila
  todos los paquetes). Las siguientes compilaciones son rapidas.
- **IMPORTANTE:** no uses un `extra_micro_ros.meta` con opciones del host
  (`-DUHANDLER_AGENT_URL`, `-DMICROROS_TRANSPORT`), rompe el build del firmware y
  la ESP32 no bootea la app (se queda en modo descarga dando ruido `0x80`).
  El transporte se configura solo con `board_microros_transport = serial`.
- Despues de flashear, si la ESP32 queda en modo descarga (ruido constante en el
  puerto), haz un reset manual cerrando y abriendo el puerto (DTR/RTS) antes de
  lanzar el agente, para que bootee la app.
- El transporte serial usa el mismo puerto USB que la consola. Cierra el monitor
  serial (`pio device monitor`) antes de lanzar el agent serial.
- El firmware publica `/servo_states` cada 0.5 s para confirmar al host que la
  ESP32 esta viva y conectada.

## Integracion con los sliders del teleop

El `teleop_node` del workspace ROS2 (lanzado por `bringup.launch.py`) publica
tambien en `/servo_commands`, mapeando los grados de cada slider al rango del
servo:

- Centro del rango de cada articulacion -> 90° (servo neutro)
- Minimo del rango -> 0°
- Maximo del rango -> 180°

Para usarlo, en una terminal lanza el micro-ROS agent y deja la ESP32 conectada,
y en otra lanza el tetleop:

```bash
# Terminal 1: agente
source /opt/ros/humble/setup.bash
source ~/microros_ws/install/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -v6

# Terminal 2: launch con sliders (abre la ventana)
source /opt/ros/humble/setup.bash
source ~/Documents/Robot_Bipedo/ros2_ws/install/setup.bash
ros2 launch robot_bringup bringup.launch.py
```

Al mover los sliders, los servos de la ESP32 se moveran.
