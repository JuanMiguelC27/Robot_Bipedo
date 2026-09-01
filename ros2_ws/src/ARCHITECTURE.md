# ARQUITECTURA DEL ROBOT BÍPEDO (ROS 2 HUMBLE)

Cómo está armado el proyecto, para que cualquiera del grupo entienda qué hace
cada parte y cómo se comunican, sin leer todo el código.

Estado: los 7 paquetes compilan y el flujo completo funciona (se probó: los
TF de la pata se publican y RViz la muestra). Falta la lógica pesada de cada
sub-área; el esqueleto ya está listo para que cada dueño la rellene.

## Paquetes y qué contiene cada uno

robot_interfaces
  Solo define los mensajes (la "regla de comunicación"). No corre nada.
  - msg/RobotCommand.msg: lo que manda el operador.
  - msg/JointTarget.msg: consigna por motor que va a control.
  - msg/JointState.msg: estado que publica control (lleva name + position).
  - srv/ y action/: carpetas creadas, vacías por ahora (.gitme adentro).

robot_description
  Define la forma física del robot en URDF/Xacro (urdf/single_leg.urdf.xacro):
  links (Base_link, Hip_Link, Knee_Link, Ankle_Link) y joints (Hip_Joint, Knee_Joint, Ankle_Joint).
  No tiene algoritmos. Sus carpetas rviz/, meshes/, urdf/ usan .gitme.

robot_kinematics
  Nodo kinematics_node: recibe /robot/command y publica /robot/joint_targets.
  Hoy reenvía; aquí va luego la cinemática directa/inversa y la locomoción.

robot_control
  Nodo control_node: recibe /robot/joint_targets, sigue la consigna con un PID
  y publica el estado y el comando de los motores. Además aplica el límite de
  ángulo por motor (clamp) y el E-Stop. Aquí van equilibrio e IMU después.
  También publica en /joint_states (estándar de ROS) para que se vea en RViz.

robot_simulation
  Nodo sim_bridge: puente simple que lleva /robot/joint_commands al controlador
  de simulación. Sus carpetas worlds/ y plugins/ están creadas para el encargado
  (van el mundo de Gazebo y los plugins reales).

robot_teleop
  Nodo teleop_node: barras deslizantes (una por motor) para mandar la orden.
  Cada slider tiene el rango del límite de ese motor. Requiere pantalla.

robot_bringup
  Launch que prende todo de una vez para demostrar la arquitectura. No tiene
  lógica, solo junta los nodos. Incluye los sliders (teleop_node).

## Nodos y tópicos (cómo se relacionan)

teleop_node --/robot/command--> kinematics_node
kinematics_node --/robot/joint_targets--> control_node
control_node --/robot/joint_states (interno, robot_interfaces)--> quien sea
control_node --/robot/joint_commands--> sim_bridge (-> Gazebo)
control_node --/joint_states (estándar)--> robot_state_publisher --TF--> RViz
cualquiera --/robot/e_stop (Bool)--> control_node (frena)

En simple:
1. El operador mueve un slider -> sale RobotCommand en /robot/command.
2. Cinemática lo pasa a consigna por motor (JointTarget) en /robot/joint_targets.
3. Control lo ejecuta y dice dónde está la pata (JointState) y qué comando manda
   (JointTarget en /robot/joint_commands).
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

El ángulo de cada motor se limita en 3 sitios con los MISMOS números:
- robot_description (URDF): <limit lower=.. upper=..>  -> la verdad física.
- robot_control (control_node): recorta el comando al límite -> seguridad.
- robot_teleop (sliders): el rango del slider = el límite -> lo ve el operador.
Si se cambia un tope, se cambia en los tres. Hoy los números están en
robot_control (clamp) y robot_teleop (sliders); el URDF los repite.

## Cómo probar (comandos detallados)

Paso 1 - Compilar (una sola vez por terminal nueva):
    cd ~/Documentos/Robot_Bipedo/ros2_ws
    source /opt/ros/humble/setup.bash
    colcon build
    source install/setup.bash

Paso 2 - Encender todo (incluye los sliders). En una terminal:
    ros2 launch robot_bringup bringup.launch.py
  Debe salir "process started with pid" para los 5 nodos y cada uno "listo".
  Se abre la ventana de sliders (teleop_node). En un servidor sin pantalla el
  teleop se queda esperando; correr en la máquina del operador.

Paso 3 - Ver que los tópicos circulan (otra terminal, con source):
    ros2 topic list
    ros2 topic echo /robot/joint_states
    rqt_graph
  rqt_graph con "Nodes/Topics (all)" muestra la cadena completa.

Paso 4 - Ver la pata en RViz (otra terminal, con source):
    ros2 run rviz2 rviz2
  En RViz: Global Options -> Fixed Frame: Base_link.
  Add -> RobotModel. En RobotModel, Description Topic: /robot_description.
  La pata debe verse entera y moverse al arrastrar los sliders.

Paso 5 - Cerrar: Ctrl+C en cada terminal.

Si RViz dice "No transform from [hip1]": el control no está publicando
/joint_states. Confirmar que el bringup corre (Paso 2) y que
ros2 topic echo /joint_states trae position con datos.

## Qué le toca a cada sub-área (pendiente)

- Cinemática: IK/FK, trayectorias, locomoción en robot_kinematics.
- Descripción: URDF completo, mundos, mallas en robot_description.
- Simulación: mundo Gazebo real y plugins en robot_simulation.
- Teleop: mejorar sliders / viewer en robot_teleop.
- Control: PID/balance/IMU real cuando haya hardware, en robot_control.

## Notas

- srv/, action/, worlds/, plugins/, urdf/, rviz/, meshes/ usan .gitme para
  aparecer en el repo aunque estén vacías; el archivo dice qué va en cada una.
- num_joints es parámetro: 3 para una pata, 6 para el robot completo. Mismos
  nodos, distinto tamaño.
