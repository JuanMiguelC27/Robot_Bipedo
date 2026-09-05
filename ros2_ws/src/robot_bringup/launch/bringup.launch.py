# Levanta TODOS los nodos del flujo para evidenciar la arquitectura:
#   robot_state_publisher (description) -> publica /robot_description + TF
#   kinematics_node  -> /robot/command -> /robot/joint_targets
#   control_node     -> /robot/joint_targets -> /robot/joint_states + /robot/joint_commands
#   sim_bridge       -> /robot/joint_commands -> /joint_group_position_controller/commands
#   teleop_node      -> SLIDERS (fuente de /robot/command; requiere pantalla/X)
# Nota: se quito test_injector para que los sliders sean la unica fuente de
# consigna y no peleen por /robot/command. En un servidor sin display el
# teleop se quedara esperando la ventana; correr en la maquina del operador.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    n = LaunchConfiguration('num_joints')
    pkg_desc = get_package_share_directory('robot_description')
    xacro_file = os.path.join(pkg_desc, 'urdf', 'Pata_Robo_Parcial_URDF_V1.2.urdf.xacro')
    rviz_config = os.path.join(pkg_desc, 'config', 'robot.rviz')
    robot_description = Command(['xacro ', xacro_file])

    description = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description':
                     ParameterValue(robot_description, value_type=str)}])

    kinematics = Node(
        package='robot_kinematics', executable='kinematics_node',
        name='kinematics_node', parameters=[{'num_joints': n}])

    control = Node(
        package='robot_control', executable='control_node',
        name='control_node', parameters=[{'num_joints': n}])

    sim = Node(
        package='robot_simulation', executable='sim_bridge',
        name='sim_bridge', output='screen')

    teleop = Node(
        package='robot_teleop', executable='teleop_node',
        name='teleop_node', parameters=[{'num_joints': n}],
        output='screen')

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz_config], output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')))

    return LaunchDescription([
        DeclareLaunchArgument('num_joints', default_value='3'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        description, kinematics, control, sim, teleop, rviz,
    ])
