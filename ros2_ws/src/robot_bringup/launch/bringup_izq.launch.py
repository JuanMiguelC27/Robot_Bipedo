"""
Launch completo para probar ÚNICAMENTE la pierna izquierda del robot bípedo.

Este launch inicia:

    1. robot_state_publisher
       - Carga el modelo URDF/Xacro de la pierna izquierda.
       - Publica /robot_description.
       - Publica las transformaciones TF del robot.

    2. kinematics_node
       - Recibe los ángulos desde /robot/command.
       - Calcula/procesa la cinemática.
       - Publica /robot/joint_targets.

    3. ik_node
       - Recibe una coordenada (x, y, z) en /robot/ik_target.
       - Calcula q1, q2, q3 (cinemática inversa algebraica).
       - Publica el resultado en /robot/ik_result.

    3.1. ik_newton_node
       - Recibe (x, y, z) + semilla (q1,q2,q3) en
         /robot/ik_newton_target.
       - Calcula q1, q2, q3 (cinemática inversa iterativa,
         método de Newton-Raphson).
       - Publica el resultado en /robot/ik_newton_result.

    3.2. ik_des_node
       - Recibe una coordenada (x, y, z) en /robot/ik_des_target.
       - Calcula q1, q2, q3 (cinemática inversa por desacople).
       - Publica el resultado en /robot/ik_des_result.

    3.3. ik_geom_node
       - Recibe una coordenada (x, y, z) en /robot/ik_geom_target.
       - Calcula q1, q2, q3 (cinemática inversa geométrica).
       - Publica el resultado en /robot/ik_geom_result.

    3.4. ik_grad_node
       - Recibe (x, y, z) + semilla (q1,q2,q3) en
         /robot/ik_grad_target.
       - Calcula q1, q2, q3 (cinemática inversa iterativa,
         gradiente descendente).
       - Publica el resultado en /robot/ik_grad_result.

    4. control_node
       - Recibe /robot/joint_targets.
       - Aplica los límites de seguridad articulares.
       - Publica /robot/joint_states.
       - Publica /robot/joint_commands.
       - HARDWARE: recibe /robot/hardware_command, revisa e-stop
         y límites, suma el offset de cada servo y publica
         /servo_commands (ESP32). Responde en
         /robot/hardware_status y convierte /servo_states a
         grados cinemáticos en /robot/hardware_state.

    5. sim_bridge
       - Conecta los comandos de articulación con la simulación.

    6. teleop_node
       - Abre la interfaz gráfica, con dos pestañas:
           - Cinemática directa: sliders/ángulos manuales,
             publica /robot/command (RViz). El botón "Aplicar a
             motores" publica /robot/hardware_command.
           - Cinemática inversa: coordenada X,Y,Z, publica
             /robot/ik_target y aplica el resultado recibido
             por /robot/ik_result.
       - Muestra la cinemática de la pierna izquierda.

    7. RViz2
       - Visualiza el modelo de la pierna y sus movimientos.

IMPORTANTE:
    Este launch está pensado para ejecutarse solo con la pierna izquierda.

    La pierna derecha y la izquierda utilizan actualmente los mismos
    nombres de nodos y tópicos, por lo que NO deben ejecutarse ambos
    bringup simultáneamente sin utilizar namespaces.

La comunicación con la ESP32 mediante robot_serial_bridge NO se inicia
desde este launch. Ese nodo se ejecuta aparte cuando sea necesario.
"""


# ======================================================================
# IMPORTACIONES DE ROS 2 LAUNCH
# ======================================================================

from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument

from launch.conditions import IfCondition

from launch.substitutions import (
    LaunchConfiguration,
    Command
)

from launch_ros.actions import Node

from launch_ros.parameter_descriptions import (
    ParameterValue
)

from ament_index_python.packages import (
    get_package_share_directory
)

import os


# ======================================================================
# CONFIGURACIÓN DE LA PIERNA IZQUIERDA
# ======================================================================

# Nombres de los joints definidos realmente en el Xacro de la
# pierna izquierda.

JOINT_NAMES = [
    'Left_Hip_Roll_Joint',
    'Left_Hip_Pitch_Joint',
    'Left_Knee_Joint'
]


# ======================================================================
# FUNCIÓN PRINCIPAL DEL LAUNCH
# ======================================================================

def generate_launch_description():

    # ------------------------------------------------------------------
    # Número de articulaciones.
    #
    # Para una sola pierna:
    #
    #     num_joints = 3
    #
    # ------------------------------------------------------------------

    n = LaunchConfiguration(
        'num_joints'
    )


    # ==================================================================
    # UBICACIÓN DEL PAQUETE robot_description
    # ==================================================================

    pkg_desc = get_package_share_directory(
        'robot_description'
    )


    # ==================================================================
    # MODELO XACRO DE LA PIERNA IZQUIERDA
    # ==================================================================

    xacro_file = os.path.join(
        pkg_desc,
        'urdf',
        'urdf_completo',
        'robot_completo.urdf.xacro'
    )


    # ------------------------------------------------------------------
    # Configuración de RViz
    # ------------------------------------------------------------------

    rviz_config = os.path.join(
        pkg_desc,
        'config',
        'robot.rviz'
    )


    # ------------------------------------------------------------------
    # Procesar el Xacro para obtener el URDF.
    # ------------------------------------------------------------------

    robot_description = Command([
        'xacro ',
        xacro_file
    ])


    # ==================================================================
    # 1. ROBOT STATE PUBLISHER
    # ==================================================================

    description = Node(
        package='robot_state_publisher',

        executable='robot_state_publisher',

        name='robot_state_publisher',

        parameters=[
            {
                'robot_description':
                    ParameterValue(
                        robot_description,
                        value_type=str
                    )
            }
        ],

        # El modelo es el robot completo (ambas piernas), pero
        # control_node solo publica en /joint_states los joints de
        # esta pierna. Se leen los estados ya completados por
        # joint_state_publisher (ver abajo) para que la otra pierna
        # también tenga TF y se dibuje en RViz.
        remappings=[
            ('joint_states', '/completo/joint_states')
        ]
    )


    # ------------------------------------------------------------------
    # 1.1. JOINT STATE PUBLISHER (completar la otra pierna)
    #
    # Toma /joint_states (los 3 joints de esta pierna, publicados por
    # control_node) y publica en /completo/joint_states los 6 joints
    # del robot completo, dejando en 0 los de la pierna que no se
    # controla.
    # ------------------------------------------------------------------

    joint_states_completo = Node(
        package='joint_state_publisher',

        executable='joint_state_publisher',

        name='joint_state_publisher',

        namespace='completo',

        parameters=[
            {
                'source_list': ['/joint_states'],
                'rate': 30
            }
        ],

        remappings=[
            ('robot_description', '/robot_description')
        ]
    )


    # ==================================================================
    # 2. NODO DE CINEMÁTICA
    # ==================================================================

    kinematics = Node(
        package='robot_kinematics',

        executable='kinematics_node',

        name='kinematics_node',

        parameters=[
            {
                'num_joints': n
            }
        ]
    )


    # ==================================================================
    # 2.1. NODO DE CINEMÁTICA INVERSA
    # ==================================================================

    ik = Node(
        package='robot_kinematics',

        executable='ik_node',

        name='ik_node',

        parameters=[
            {
                'leg_side': 'left'
            }
        ]
    )


    # ==================================================================
    # 2.2. NODO DE CINEMÁTICA INVERSA (NEWTON-RAPHSON)
    # ==================================================================

    ik_newton = Node(
        package='robot_kinematics',

        executable='ik_newton_node',

        name='ik_newton_node',

        parameters=[
            {
                'leg_side': 'left'
            }
        ]
    )


    # ==================================================================
    # 2.3. NODO DE CINEMÁTICA INVERSA (DESACOPLE)
    # ==================================================================

    ik_des = Node(
        package='robot_kinematics',

        executable='ik_des_node',

        name='ik_des_node',

        parameters=[
            {
                'leg_side': 'left'
            }
        ]
    )


    # ==================================================================
    # 2.4. NODO DE CINEMÁTICA INVERSA (GEOMÉTRICO)
    # ==================================================================

    ik_geom = Node(
        package='robot_kinematics',

        executable='ik_geom_node',

        name='ik_geom_node',

        parameters=[
            {
                'leg_side': 'left'
            }
        ]
    )


    # ==================================================================
    # 2.5. NODO DE CINEMÁTICA INVERSA (GRADIENTE DESCENDENTE)
    # ==================================================================

    ik_grad = Node(
        package='robot_kinematics',

        executable='ik_grad_node',

        name='ik_grad_node',

        parameters=[
            {
                'leg_side': 'left'
            }
        ]
    )


    # ==================================================================
    # 3. NODO DE CONTROL
    # ==================================================================

    control = Node(
        package='robot_control',

        executable='control_node',

        name='control_node',

        parameters=[
            {
                # Número de articulaciones de esta pierna.
                'num_joints': n,

                # Nombres reales de los joints del URDF izquierdo.
                'joint_names': JOINT_NAMES,

                # ------------------------------------------------------
                # NO se pasan límites articulares desde el launch.
                #
                # control_node.py es responsable de aplicar los
                # límites internos de seguridad.
                # ------------------------------------------------------
            }
        ]
    )


    # ==================================================================
    # 4. PUENTE CON LA SIMULACIÓN
    # ==================================================================

    sim = Node(
        package='robot_simulation',

        executable='sim_bridge',

        name='sim_bridge',

        output='screen'
    )


    # ==================================================================
    # 5. INTERFAZ DE TELEOPERACIÓN
    # ==================================================================

    teleop = Node(
        package='robot_teleop',

        executable='teleop_node',

        name='teleop_node',

        parameters=[
            {
                # Número de articulaciones controladas.
                'num_joints': n,

                # ------------------------------------------------------
                # DIFERENCIA FUNDAMENTAL CON EL LAUNCH DERECHO:
                #
                # Indicamos que esta interfaz corresponde a la
                # PIERNA IZQUIERDA.
                #
                # teleop_node.py utilizará:
                #
                #     forward_kinematics_left()
                #
                # y mostrará:
                #
                #     TELEOPERACIÓN PIERNA IZQUIERDA
                # ------------------------------------------------------
                'leg_side': 'left',

                # ------------------------------------------------------
                # Los límites visuales permanecen definidos dentro
                # de teleop_node.py.
                # ------------------------------------------------------
            }
        ],

        output='screen'
    )


    # ==================================================================
    # 6. RVIZ2
    # ==================================================================

    rviz = Node(
        package='rviz2',

        executable='rviz2',

        name='rviz2',

        arguments=[
            '-d',
            rviz_config
        ],

        output='screen',

        # RViz se ejecuta por defecto.
        #
        # Puede desactivarse con:
        #
        #     use_rviz:=false
        #
        condition=IfCondition(
            LaunchConfiguration('use_rviz')
        )
    )


    # ==================================================================
    # RETORNAR TODOS LOS NODOS DEL LAUNCH
    # ==================================================================

    return LaunchDescription([

        # Número de articulaciones.
        DeclareLaunchArgument(
            'num_joints',
            default_value='3'
        ),

        # RViz activado por defecto.
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true'
        ),

        # Nodos.
        description,
        joint_states_completo,
        kinematics,
        ik,
        ik_newton,
        ik_des,
        ik_geom,
        ik_grad,
        control,
        sim,
        teleop,
        rviz,
    ])
