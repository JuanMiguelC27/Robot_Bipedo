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

    3. control_node
       - Recibe /robot/joint_targets.
       - Aplica los límites de seguridad articulares.
       - Publica /robot/joint_states.
       - Publica /robot/joint_commands.

    4. sim_bridge
       - Conecta los comandos de articulación con la simulación.

    5. teleop_node
       - Abre la interfaz gráfica.
       - Permite controlar las tres articulaciones.
       - Publica /robot/command.
       - Publica /servo_commands.
       - Muestra la cinemática directa de la pierna izquierda.

    6. RViz2
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
        'urdf_izq',
        'pata_izq.urdf.xacro'
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
        kinematics,
        control,
        sim,
        teleop,
        rviz,
    ])
