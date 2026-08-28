# ESQUELETO de launch de simulacion. Prende el puente que lleva la
# consigna de control a un controlador de Gazebo. El encargado de
# simulacion agrega aqui el mundo Gazebo y el ros_gz_bridge.
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bridge = Node(
        package='robot_simulation',
        executable='sim_bridge',
        output='screen')
    return LaunchDescription([bridge])
