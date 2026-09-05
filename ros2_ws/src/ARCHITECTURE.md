# ARQUITECTURA DEL ROBOT BÍPEDO (ROS 2 HUMBLE)

Cómo está armado el proyecto, para que cualquiera del grupo entienda qué hace
cada parte y cómo se comunican, sin leer todo el código.

Estado: los 8 paquetes compilan y el flujo completo funciona por pata (se
probó: los TF de cada pata se publican y RViz la muestra). Falta la lógica
pesada de cada sub-área; el esqueleto ya está listo para que cada dueño la
rellene.

## Paquetes y qué contiene cada uno

robot_interfaces
  Solo define los mensajes (la "regla de comunicación"). No corre nada.
  - msg/RobotCommand.msg: lo que manda el operador.
  - msg/JointTarget.msg: consigna por motor que va a control.
  - msg/JointState.msg: estado que publica control (lleva name + position).
  - srv/ y action/: carpetas creadas, vacías por ahora (.gitme adentro).

robot_description
  Define la forma física del robot en URDF/Xacro. Ya no es una sola pata
  genérica: hay un URDF real por lado (exportado de SolidWorks), cada uno
  con su wrapper xacro que agrega un frame "world" fijo (rotado 90° para
  verse vertical en RViz) y con sus propias mallas:
    urdf/urdf_der/pata_der.urdf.xacro -> Pata_Der_Robo_Parcial_URDF_V2.2.urdf
      Links: Base_link, Right_Hip_Roll_Link, Right_Hip_Pitch_Link,
      Right_Knee_Link. Joints: Right_Hip_Roll_Joint, Right_Hip_Pitch_Joint,
      Right_Knee_Joint.
    urdf/urdf_izq/pata_izq.urdf.xacro -> Pata_Izq_Robo_Parcial_URDF_V2.5.urdf
      Links: Base_link, Left_Hip_Roll_Link, Left_Hip_Pitch_Link,
      Left_Knee_Link. Joints: Left_Hip_Roll_Joint, Left_Hip_Pitch_Joint,
      Left_Knee_Joint.
  Límites reales (en radianes, iguales para ambos lados, solo cambia el
  signo del eje): Hip_Roll [-0.34907, 2.2689], Hip_Pitch [-2.0071, 2.0071],
  Knee [-2.0071, 2.0071]. Ya no hay Ankle_Joint (el modelo viejo de 4 DOF
  con tobillo quedó obsoleto). No hay todavía un torso/pelvis que una las
  dos patas en un solo biped: son dos modelos independientes, cada uno con
  su propio Base_link raíz. No tiene algoritmos.

robot_kinematics
  Nodo kinematics_node: recibe /robot/command y publica /robot/joint_targets.
  Usa kinem_leg_gen.py para calcular la cinemática directa (forward kinematics)
  con parámetros DH. Hoy reenvía; aquí va luego la cinemática inversa,
  trayectorias y coordinación de patas.

robot_control
  Nodo control_node: recibe /robot/joint_targets.
  - Modo verify_mode (por defecto True): solo aplica límites articulares y
    publica el estado tal cual en /joint_states. Sin PID. Sirve para confirmar
    que lo que entra por los sliders se refleja idéntico en RViz.
  - Modo PID (verify_mode=False): activa el PID para control avanzado.
  Parámetros dinámicos:
    - joint_names: nombres reales de los 3 joints (default = pierna derecha:
      Right_Hip_Roll_Joint, Right_Hip_Pitch_Joint, Right_Knee_Joint).
      robot_bringup lo sobreescribe con los Left_* al lanzar la pata
      izquierda (esto sí lo pone el launch, porque son joints distintos
      por pata).
    - joint_limits_lower / joint_limits_upper: límite INTERNO real del motor,
      en radianes. Vive únicamente en control_node.py (bringup ya NO lo
      sobreescribe): para cambiarlo se edita este .py o se usa
      "ros2 param set /control_node ..." en vivo. Es el mismo para las dos
      patas. Hoy: [-1.91986, -2.0071, -2.0071] a [2.2689, 2.0071, 2.0071] —
      OJO, cadera-roll (-1.91986 rad / -110°) no es el nominal del URDF
      (-0.34907 rad / -20°): es un offset físico temporal del motor: cuando
      se recalibre, hay que volver a poner -0.34907 acá.
    - verify_mode: True = directo, False = PID.
  También publica en /joint_states (estándar) para que se vea en RViz.

robot_simulation
  Nodo sim_bridge: puente simple que lleva /robot/joint_commands al controlador
  de simulación. Sus carpetas worlds/ y plugins/ están creadas para el encargado
  (van el mundo de Gazebo y los plugins reales).

robot_teleop
  Nodo teleop_node: barras deslizantes en grados (una por motor) + textbox
  para escribir el ángulo a mano ("Aplicar ángulos"), más la matriz 0A4 de
  cinemática directa. Es el mismo nodo genérico para las dos patas (no sabe
  de nombres de joints, solo de cuántos hay); lo que cambia entre pata
  derecha e izquierda es el rango de cada slider.
  Parámetros dinámicos:
    - joint_limits_lower_deg / joint_limits_upper_deg: límite VISUAL (rango
      del slider + validación del textbox), en grados. Vive únicamente en
      teleop_node.py (bringup ya NO lo sobreescribe): para cambiarlo se edita
      este .py, o "ros2 param set /teleop_node ..." (pero el slider ya
      dibujado no lo toma en caliente, hay que relanzar). Debe reflejar
      SIEMPRE el mismo límite que control_node.py, convertido a grados. Hoy:
      [-110, -115, -115] a [130, 115, 115] (mismos para ambas patas).
  Requiere pantalla/X.
  Nota: no se tocó nada de cómo se envía la orden a los servos
  (/servo_commands, home_angle_deg): eso lo consume tal cual
  robot_serial_bridge hacia la ESP32 (micro-ROS/PlatformIO), que sigue
  corriendo aparte, sin cambios.

robot_bringup
  Un launch POR PATA (no uno conjunto): cada uno prende el mismo flujo de
  antes -- robot_state_publisher, kinematics_node, control_node, sim_bridge,
  teleop_node (con sus sliders/textbox) y rviz2 -- pero apuntando solo al
  URDF y joint_names de esa pata.
    launch/bringup_der.launch.py -> pata_der.urdf.xacro, joints Right_*.
    launch/bringup_izq.launch.py -> pata_izq.urdf.xacro, joints Left_*.
  Los límites articulares (interno y visual) NO se pasan desde acá: eso es
  tarea de cada nodo (control_node.py / teleop_node.py), ver esas secciones
  más arriba. bringup solo decide qué joints existen en cada pata.
  Usan los mismos nombres de tópico/nodo que la versión de una sola pata (no
  hay namespacing): están pensados para correr UNA PATA A LA VEZ. Si algún
  día se corren las dos terminales al mismo tiempo, sus tópicos (/robot/
  command, /joint_states, etc.) y nodos (control_node, teleop_node, ...) se
  pisarían entre sí.
  El puente serial con la ESP32 (robot_serial_bridge) no está incluido acá:
  se corre aparte, apuntando al puerto de la ESP32 de la pata que se esté
  probando (ver sección "Cómo probar").

## Nodos y tópicos (cómo se relacionan)

teleop_node --/robot/command--> kinematics_node
kinematics_node --/robot/joint_targets--> control_node
control_node --/robot/joint_states (interno, robot_interfaces)--> quien sea
control_node --/joint_states (estándar)--> robot_state_publisher --TF--> RViz
control_node --/robot/joint_commands--> sim_bridge (-> Gazebo)
cualquiera --/robot/e_stop (Bool)--> control_node (frena)

En simple:
1. El operador mueve un slider -> sale RobotCommand en /robot/command.
2. Cinemática lo pasa a consigna por motor (JointTarget) en /robot/joint_targets.
3. Control la verifica, aplica límites y publica el estado (JointState).
4. robot_state_publisher usa /joint_states para calcular los TF y RViz dibuja
   la pata moviéndose.
5. sim_bridge lleva el comando a Gazebo.

## Mensajes

RobotCommand: mode (0 parado / 1 caminar), position[], velocity[]
JointTarget:  position[], velocity[]   (consigna que va a control)
JointState:   name[], position[], velocity[], effort[], battery  (estado real)
Nota: el estado también se publica en /joint_states como sensor_msgs/JointState
(estándar) porque robot_state_publisher solo entiende ese tópico.

## Límites de los motores

El ángulo de cada motor se limita en 3 sitios, y cada uno vive en SU PROPIO
nodo (bringup no reparte estos números, solo arma el grafo de nodos):
- robot_description (URDF): <limit lower=.. upper=..>  -> la verdad física
  de diseño (referencia; ningún nodo la lee en tiempo real).
- robot_control (control_node.py): límite INTERNO real del motor, en
  radianes -> lo aplica siempre (clamp de seguridad), el operador no lo ve.
- robot_teleop (teleop_node.py): límite VISUAL, en grados -> el rango del
  slider y la validación del textbox; es lo único que ve/ajusta el operador.

Estos dos DEBEN mantenerse en fase (mismo ángulo, control en radianes y
teleop en grados) aunque no coincidan con el nominal del URDF (por ejemplo,
hoy cadera-roll tiene un offset físico real de -110° en vez de los -20° de
diseño). Para cambiarlos, en cualquiera de las dos patas (los valores son
los mismos para ambas):
- Editando el .py: joint_limits_lower/upper en
  robot_control/robot_control/control_node.py, y joint_limits_lower_deg/
  upper_deg en robot_teleop/robot_teleop/teleop_node.py. Hay que
  recompilar/relanzar para que tome efecto.
- En vivo, sin tocar archivos (control lo aplica al instante; teleop
  necesita relanzar igual, el slider ya dibujado no cambia de rango solo):
    ros2 param set /control_node joint_limits_lower "[-1.91986, -2.0071, -2.0071]"
    ros2 param set /control_node joint_limits_upper "[2.2689, 2.0071, 2.0071]"
    ros2 param set /teleop_node joint_limits_lower_deg "[-110.0, -115.0, -115.0]"
    ros2 param set /teleop_node joint_limits_upper_deg "[130.0, 115.0, 115.0]"

## Modos de control

- verify_mode=True (por defecto): el valor del slider llega directo a RViz,
  solo con clamp de límites. Sin PID, sin filtros. Para verificación.
- verify_mode=False: se activa el PID para control avanzado, trayectorias,
  cinemática inversa, etc.

## Cómo probar (comandos detallados)

Paso 1 - Compilar (una sola vez por terminal nueva):
    cd ~/Documentos/Robot_Bipedo/ros2_ws
    source /opt/ros/humble/setup.bash
    colcon build
    source install/setup.bash

Paso 2 - Encender una pata (incluye los sliders). Elegí UNA terminal por
pata (no las dos a la vez, ver nota de robot_bringup más arriba):
    ros2 launch robot_bringup bringup_der.launch.py     # pierna derecha
    ros2 launch robot_bringup bringup_izq.launch.py     # pierna izquierda
  Debe salir "process started with pid" para los 5 nodos (description,
  kinematics, control, sim, teleop) y cada uno "listo". Se abre la ventana
  de sliders + textbox (teleop_node) y RViz con la pata elegida. En un
  servidor sin pantalla el teleop se queda esperando; correr en la máquina
  del operador. Para apagar rviz2 (por ejemplo si vas a usar el tuyo propio
  del Paso 4): agregar 'use_rviz:=false' al final del comando.

Paso 3 - Ver que los tópicos circulan (otra terminal, con source):
    ros2 topic list
    ros2 topic echo /joint_states
    rqt_graph
  rqt_graph con "Nodes/Topics (all)" muestra la cadena completa.

Paso 4 - Ver la pata en RViz (ya se abre sola en el Paso 2; si querés una
aparte, otra terminal con source):
    ros2 run rviz2 rviz2
  En RViz: Global Options -> Fixed Frame: world.
  Add -> RobotModel. En RobotModel, Description Topic: /robot_description.
  La pata debe verse entera y moverse al arrastrar los sliders.

Paso 5 - Cambiar límites en vivo (sin recompilar; control los aplica al
toque, teleop necesita relanzar para que el slider tome el rango nuevo):
    ros2 param set /control_node joint_limits_lower "[-1.91986, -2.0071, -2.0071]"
    ros2 param set /control_node joint_limits_upper "[2.2689, 2.0071, 2.0071]"
    ros2 param set /teleop_node joint_limits_lower_deg "[-110.0, -115.0, -115.0]"
    ros2 param set /teleop_node joint_limits_upper_deg "[130.0, 115.0, 115.0]"

Paso 6 (opcional) - Conectar la ESP32 real de esa pata (aparte del
bringup, en otra terminal con source):
    ros2 run robot_serial_bridge serial_bridge -p port:=/dev/ttyUSBx
  Cambiar /dev/ttyUSBx por el puerto real de la ESP32 de la pata que estás
  probando. El firmware (PlatformIO/micro-ROS) no cambió: sigue leyendo
  /servo_commands igual que antes.

Paso 7 - Cerrar: Ctrl+C en cada terminal.

Si RViz dice "No transform from [world]" o no aparece la pata: el control
no está publicando /joint_states. Confirmar que el bringup corre (Paso 2) y
que ros2 topic echo /joint_states trae position con datos.

## Qué le toca a cada sub-área (pendiente)

- Cinemática: IK/FK, trayectorias, locomoción en robot_kinematics.
- Descripción: URDF completo, mundos, mallas en robot_description.
- Simulación: mundo Gazebo real y plugins en robot_simulation.
- Teleop: mejorar sliders / viewer en robot_teleop.
- Control: PID/balance/IMU real cuando haya hardware, en robot_control.

## Notas

- num_joints es parámetro: 3 para una pata. Cuando exista un URDF combinado
  de las dos patas (torso/pelvis común) esto sube a 6; hoy no existe ese
  URDF combinado, cada pata es un modelo independiente.
- kinem_leg_gen.py implementa la cinemática directa con Denavit-Hartenberg.
  Funciones: forward_kinematics(q1, q2, q3) y end_effector_position(q1, q2, q3).
- Cada URDF real (SolidWorks) se incluye vía un wrapper xacro
  (pata_der.urdf.xacro / pata_izq.urdf.xacro) para compatibilidad con ROS2 y
  para agregar el frame "world" que usa RViz como Fixed Frame.
