# Robot Bípedo — Plataforma de Desarrollo

Repositorio principal para el desarrollo de software, control, simulación e integración del proyecto **Robot Bípedo**.

El proyecto consiste en el desarrollo de una plataforma robótica bípeda, equipada con dos piernas, seis grados de libertad y ruedas integradas en los pies. El sistema contempla un movimiento de caminata dinámica.

Este repositorio concentra los recursos de programación y desarrollo necesarios para la implementación, simulación, control e integración del robot.

---

## 📁 Estructura del repositorio

```text
Robot_bipedoCode/
│
├── README.md
├── .gitignore
├── Robot_bipedoCode.code-workspace
│
├── ros2_ws/
│   └── src/
│       ├── robot_description/
│       ├── robot_interfaces/
│       ├── robot_kinematics/
│       ├── robot_control/
│       ├── robot_simulation/
│       └── robot_teleop/
│
├── matlab/
│   ├── cinematica/
│   ├── control/
│   ├── dinamica/
│   └── simulaciones/
│
├── scripts/
│
├── tests/
│
└── docs/
    ├── arquitectura/
    ├── cinematica/
    ├── control/
    └── simulacion/
```

> La estructura puede evolucionar durante el desarrollo. Los paquetes y directorios se crearán o modificarán conforme aparezcan nuevas necesidades del proyecto.

---

## 🤖 ROS 2

La carpeta `ros2_ws/` contiene el **Workspace de ROS 2** del proyecto.

Dentro de `ros2_ws/src/` se desarrollarán los diferentes paquetes encargados de las funcionalidades del robot.

### `robot_description`

Contendrá la descripción del robot:

- Modelos URDF/Xacro.
- Links y joints.
- Parámetros físicos.
- Mallas 3D.
- Configuración para RViz.
- Archivos de lanzamiento relacionados con la descripción del robot.

### `robot_interfaces`

Contendrá las interfaces de comunicación utilizadas por los diferentes componentes de ROS 2:

- Messages (`msg`).
- Services (`srv`).
- Actions (`action`).

Este paquete permitirá establecer interfaces comunes entre los distintos módulos del sistema.

### `robot_kinematics`

Contendrá los algoritmos relacionados con:

- Cinemática directa.
- Cinemática inversa.
- Generación de trayectorias.
- Generación de movimientos.
- Locomoción.
- Coordinación de ambas piernas.

### `robot_control`

Contendrá los algoritmos de control del robot:

- Control de motores.
- Control de posición, velocidad y torque.
- Controladores PID u otros.
- Control del equilibrio.
- Procesamiento de sensores.
- Integración de la IMU.
- Coordinación de los actuadores.
- Evaluación de estrategias avanzadas como MPC, si resulta viable.

### `robot_simulation`

Contendrá los recursos necesarios para la simulación:

- Gazebo.
- Mundos de simulación.
- Modelos y obstáculos.
- Configuraciones de sensores.
- Plugins.
- Archivos de lanzamiento.
- Pruebas de locomoción, equilibrio y control.

### `robot_teleop`

Contendrá los sistemas relacionados con la interacción del operador:

- Joystick.
- Gamepad.
- Interfaces de usuario.
- Comandos de teleoperación.
- Selección de modos de funcionamiento.
- Visualización del estado del robot.
- Mecanismos de parada desde software.

---

## 🧮 MATLAB

La carpeta `matlab/` estará destinada al desarrollo y análisis matemático realizado en MATLAB.

Se utilizará principalmente para:

- Desarrollo y validación de modelos matemáticos.
- Cinemática.
- Dinámica.
- Diseño y análisis de controladores.
- Simulaciones.
- Análisis de estabilidad.
- Validación de algoritmos antes de su implementación en ROS 2.

Los desarrollos de MATLAB que posteriormente sean implementados como nodos o componentes del sistema podrán utilizarse como referencia para su implementación final.

---

## 🧰 Scripts

La carpeta `scripts/` contendrá herramientas auxiliares que no correspondan directamente a un paquete ROS 2.

Puede incluir:

- Scripts de Python.
- Scripts Bash.
- Herramientas de automatización.
- Procesamiento de datos.
- Conversión de archivos.
- Herramientas de configuración o diagnóstico.

Los scripts que formen parte directamente de un paquete ROS 2 deberán permanecer dentro del paquete correspondiente.

---

## 🧪 Tests

La carpeta `tests/` estará destinada a pruebas generales del proyecto.

Se podrán incluir:

- Pruebas unitarias.
- Pruebas de algoritmos.
- Pruebas de comunicación.
- Pruebas de integración.
- Validación de componentes.
- Resultados y herramientas de verificación.

Las pruebas específicas de un paquete ROS 2 podrán mantenerse dentro del propio paquete.

---

## 📚 Documentación

La carpeta `docs/` contendrá documentación técnica relacionada con el desarrollo del software.

Se organizará por áreas, por ejemplo:

```text
docs/
├── arquitectura/
├── cinematica/
├── control/
└── simulacion/
```

Aquí podrán almacenarse:

- Decisiones de arquitectura.
- Diagramas.
- Documentación de interfaces.
- Procedimientos técnicos.
- Resultados de pruebas.
- Notas de implementación.
- Información necesaria para facilitar la incorporación de nuevos integrantes.

La documentación general del proyecto y las instrucciones principales de uso deberán mantenerse actualizadas en este repositorio.

---

## 🧩 Relación con las áreas de desarrollo

La estructura del repositorio está alineada con las áreas definidas para el proyecto:

| Área | Componentes principales |
|---|---|
| Arquitectura ROS 2 | `ros2_ws/`, `robot_interfaces/` |
| Cinemática y locomoción | `robot_kinematics/`, `matlab/cinematica/` |
| Simulación | `robot_description/`, `robot_simulation/` |
| Interfaz y teleoperación | `robot_teleop/` |
| Control | `robot_control/`, `matlab/control/` |
| Integración y pruebas | `tests/` y pruebas dentro de cada paquete |

---

## 🛠️ Tecnologías principales

El desarrollo podrá involucrar las siguientes tecnologías:

- **ROS 2**
- **Python**
- **C/C++**
- **MATLAB / Simulink**
- **Gazebo**
- **RViz**
- **URDF / Xacro**
- **Git / GitHub**
- **Micro-ROS**, cuando corresponda a la integración con sistemas embebidos.

---

## 🚀 Flujo general de desarrollo

El desarrollo seguirá, de manera general, el siguiente flujo:

```text
Modelado y análisis
       │
       ├── MATLAB
       └── Python
              │
              ▼
        Implementación
              │
              ▼
           ROS 2
              │
       ┌──────┴──────┐
       ▼             ▼
   Simulación     Hardware
    Gazebo       del robot
       │             │
       └──────┬──────┘
              ▼
          Validación
              │
              ▼
        Integración
```

Los algoritmos podrán desarrollarse y validarse inicialmente mediante herramientas matemáticas y simulación antes de ser integrados al robot físico.

---

## 🌿 Control de versiones

El proyecto utiliza **Git** para el control de versiones y **GitHub** como repositorio remoto.

Se recomienda trabajar mediante ramas para evitar realizar cambios directamente sobre `main`.

Ejemplo:

```text
main
│
├── feature/robot-description
├── feature/kinematics
├── feature/control
├── feature/simulation
└── feature/teleoperation
```

Los nombres y estrategia de ramas podrán ajustarse posteriormente según las necesidades del equipo.

---

## ⚠️ Archivos que no deben subirse

Los archivos generados automáticamente por ROS 2 no deben formar parte del repositorio:

```text
ros2_ws/build/
ros2_ws/install/
ros2_ws/log/
```

Estos directorios se generan localmente al compilar el Workspace.

De igual manera, archivos temporales, cachés, entornos virtuales y archivos específicos de cada máquina deberán excluirse mediante `.gitignore`.

---

## 👥 Desarrollo colaborativo

Cada integrante deberá trabajar sobre la funcionalidad correspondiente a su área y mantener una estructura de archivos coherente con la arquitectura definida.

Antes de integrar cambios al proyecto se deberán verificar:

1. Compilación correcta.
2. Ausencia de errores conocidos.
3. Compatibilidad con los demás paquetes.
4. Funcionamiento de las pruebas correspondientes.
5. Actualización de la documentación cuando sea necesario.

El objetivo es mantener un repositorio organizado, reproducible y escalable durante todo el desarrollo del robot.