"""
Launch completo para probar ÚNICAMENTE la pierna derecha del robot bípedo.

Este launch inicia:

    1. robot_state_publisher
       - Carga el modelo URDF/Xacro de la pierna derecha.
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
       - Muestra la cinemática directa de la pierna derecha.

    6. RViz2
       - Visualiza el modelo de la pierna y sus movimientos.

IMPORTANTE:
    Este launch está pensado para ejecutarse solo con la pierna derecha.

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
# CONFIGURACIÓN DE LA PIERNA DERECHA
# ======================================================================

# Nombres de los joints definidos realmente en el Xacro de la
# pierna derecha.
#
# Estos nombres son importantes para que robot_control y RViz
# trabajen con las articulaciones correctas.

JOINT_NAMES = [
    'Right_Hip_Roll_Joint',
    'Right_Hip_Pitch_Joint',
    'Right_Knee_Joint'
]


# ======================================================================
# FUNCIÓN PRINCIPAL DEL LAUNCH
# ======================================================================

def generate_launch_description():

    # ------------------------------------------------------------------
    # Parámetro que permite indicar cuántas articulaciones se controlan.
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
    # MODELO XACRO DE LA PIERNA DERECHA
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
    #
    # Command ejecuta:
    #
    #     xacro robot_completo.urdf.xacro
    #
    # y genera el contenido del robot_description.
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

                # Nombres reales de los joints del URDF derecho.
                'joint_names': JOINT_NAMES,

                # ------------------------------------------------------
                # IMPORTANTE:
                #
                # NO se pasan aquí los límites articulares.
                #
                # Los límites de seguridad están definidos dentro
                # de control_node.py.
                #
                # Esto evita tener varias fuentes diferentes para
                # los límites del robot.
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
                # Indica explícitamente que esta interfaz corresponde
                # a la PIERNA DERECHA.
                #
                # teleop_node.py utiliza este parámetro para:
                #
                #   - Mostrar "PIERNA DERECHA" en la interfaz.
                #   - Utilizar forward_kinematics_right().
                # ------------------------------------------------------
                'leg_side': 'right',

                # ------------------------------------------------------
                # Los límites visuales de los sliders y entradas
                # manuales se mantienen definidos dentro de
                # teleop_node.py.
                #
                # No se pasan desde este launch.
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

        # RViz puede activarse/desactivarse mediante:
        #
        #     use_rviz:=true
        #
        # o:
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

        # Activar RViz por defecto.
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true'
        ),

        # Nodos.
        description,
        joint_states_completo,
        kinematics,
        control,
        sim,
        teleop,
        rviz,
    ])
