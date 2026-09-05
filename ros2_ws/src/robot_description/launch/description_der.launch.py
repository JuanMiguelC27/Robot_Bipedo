from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('robot_description')
    xacro_file = os.path.join(pkg, 'urdf', 'urdf_der', 'pata_der.urdf.xacro')
    robot_description_content = Command(['xacro ', xacro_file])

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description':
                     ParameterValue(robot_description_content, value_type=str)}])

    jsp_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        parameters=[{'use_gui': True}])

    return LaunchDescription([rsp, jsp_gui])
