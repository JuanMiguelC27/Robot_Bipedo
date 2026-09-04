from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    # =========================================================
    # PARÁMETROS
    # =========================================================

    n = LaunchConfiguration('num_joints')

    # =========================================================
    # ROBOT DESCRIPTION
    # =========================================================

    pkg_desc = get_package_share_directory('robot_description')

    xacro_file = os.path.join(
        pkg_desc,
        'urdf',
        'Pata_Robo_Parcial_URDF_V2.4.xacro'
    )

    rviz_config = os.path.join(
        pkg_desc,
        'config',
        'robot.rviz'
    )

    robot_description = Command([
        'xacro ',
        xacro_file
    ])

    # =========================================================
    # ROBOT STATE PUBLISHER
    # =========================================================

    description = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',

        parameters=[
            {
                'robot_description': ParameterValue(
                    robot_description,
                    value_type=str
                )
            }
        ],

        output='screen'
    )

    # =========================================================
    # CINEMÁTICA
    #
    # /robot/command
    #       ↓
    # kinematics_node
    #       ↓
    # /robot/joint_targets
    # =========================================================

    kinematics = Node(
        package='robot_kinematics',
        executable='kinematics_node',
        name='kinematics_node',

        parameters=[
            {
                'num_joints': n
            }
        ],

        output='screen'
    )

    # =========================================================
    # CONTROL
    #
    # /robot/joint_targets
    #       ↓
    # control_node
    #       ↓
    # /robot/joint_states
    # /robot/joint_commands
    # /joint_states
    # =========================================================

    control = Node(
        package='robot_control',
        executable='control_node',
        name='control_node',

        parameters=[
            {
                'num_joints': n,

                'joint_names': [
                    'Right_Hip_Roll_Joint',
                    'Right_Hip_Pitch_Joint',
                    'Right_Knee_Joint',

                    'Left_Hip_Roll_Joint',
                    'Left_Hip_Pitch_Joint',
                    'Left_Knee_Joint'
                ],

                'joint_limits_lower': [
                    -0.174533,   # Right Hip Roll
                    -1.8326,   # Right Hip Pitch
                    -1.8326,   # Right Knee

                    -0.174533,   # Left Hip Roll
                    -1.8326,   # Left Hip Pitch
                    -1.8326    # Left Knee
                ],

                'joint_limits_upper': [
                    2.0944,    # Right Hip Roll
                    1.8326,    # Right Hip Pitch
                    1.8326,    # Right Knee

                    2.0944,    # Left Hip Roll
                    1.8326,    # Left Hip Pitch
                    1.8326     # Left Knee
                ]
            }
        ],

        output='screen'
    )

    # =========================================================
    # SIMULACIÓN
    #
    # /robot/joint_commands
    #       ↓
    # sim_bridge
    #       ↓
    # controlador de simulación
    # =========================================================

    sim = Node(
        package='robot_simulation',
        executable='sim_bridge',
        name='sim_bridge',

        output='screen'
    )

    # =========================================================
    # TELEOPERACIÓN
    #
    # 6 SLIDERS
    #
    # Derecha:
    #   0 -> Right Hip Roll
    #   1 -> Right Hip Pitch
    #   2 -> Right Knee
    #
    # Izquierda:
    #   3 -> Left Hip Roll
    #   4 -> Left Hip Pitch
    #   5 -> Left Knee
    #
    # /robot/command
    # =========================================================

    teleop = Node(
        package='robot_teleop',
        executable='teleop_node',
        name='teleop_node',

        parameters=[
            {
                'num_joints': n
            }
        ],

        output='screen'
    )

    # =========================================================
    # RVIZ
    # =========================================================

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',

        arguments=[
            '-d',
            rviz_config
        ],

        output='screen',

        condition=IfCondition(
            LaunchConfiguration('use_rviz')
        )
    )

    # =========================================================
    # LAUNCH DESCRIPTION
    # =========================================================

    return LaunchDescription([

        # Robot completo = 6 articulaciones
        DeclareLaunchArgument(
            'num_joints',
            default_value='6',
            description='Número de articulaciones del robot'
        ),

        # RViz activado por defecto
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Abrir RViz'
        ),

        # Nodos
        description,
        kinematics,
        control,
        sim,
        teleop,
        rviz
    ])
