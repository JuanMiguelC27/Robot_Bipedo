from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('robot_description')
    xacro_file = os.path.join(pkg, 'urdf', 'urdf_izq', 'pata_izq.urdf.xacro')
    robot_description_content = Command(['xacro ', xacro_file])

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description':
                     ParameterValue(robot_description_content, value_type=str)}])

    tf_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'Base_link', 'base_footprint'],
        output='screen')

    # Gazebo (gazebo_ros spawn_entity) no procesa xacro: se saca el robot del
    # topico /robot_description (ya expandido por robot_state_publisher) en
    # vez de pasarle el archivo .xacro directamente.
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'pata_izq',
            '-topic', 'robot_description',
            '-x', '0', '-y', '0', '-z', '0.0'
        ],
        output='screen')

    return LaunchDescription([rsp, tf_base, spawn_entity])
