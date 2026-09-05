# Levanta el flujo completo para probar SOLO la pierna izquierda:
#   robot_state_publisher (pata_izq.urdf.xacro) -> publica /robot_description + TF
#   kinematics_node  -> /robot/command -> /robot/joint_targets
#   control_node     -> /robot/joint_targets -> /robot/joint_states + /robot/joint_commands
#   sim_bridge       -> /robot/joint_commands -> /joint_group_position_controller/commands
#   teleop_node      -> SLIDERS + textbox (fuente de /robot/command; requiere pantalla/X)
# Usa los MISMOS nombres de nodo/topico que la version de una sola pata (no
# hay namespacing): correr esto y bringup_der.launch.py al mismo tiempo haria
# que se pisen los topicos. Se pensaron para correr una pata a la vez.
# El puente serial con la ESP32 (robot_serial_bridge) no esta aqui: se corre
# aparte, a mano, apuntando al puerto de la ESP32 de esta pata:
#   ros2 run robot_serial_bridge serial_bridge -p port:=/dev/ttyUSBx
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

# Nombres reales de los joints de la pierna izquierda (esto SI cambia por
# pata: son los joints que existen en pata_izq.urdf.xacro). Los limites
# articulares NO se fijan aca: viven adentro de cada nodo (control_node.py
# = limite interno en radianes, teleop_node.py = limite visual en grados) y
# son iguales para las dos patas, asi que no hace falta pasarlos por launch.
# Para cambiarlos: editar el .py de ese nodo, o "ros2 param set" en vivo.
JOINT_NAMES = ['Left_Hip_Roll_Joint', 'Left_Hip_Pitch_Joint', 'Left_Knee_Joint']


def generate_launch_description():
    n = LaunchConfiguration('num_joints')
    pkg_desc = get_package_share_directory('robot_description')
    xacro_file = os.path.join(pkg_desc, 'urdf', 'urdf_izq', 'pata_izq.urdf.xacro')
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
        name='control_node',
        parameters=[{
            'num_joints': n,
            'joint_names': JOINT_NAMES,
            # joint_limits_lower/upper NO se pasan aca: los define
            # control_node.py (limite interno real del motor).
        }])

    sim = Node(
        package='robot_simulation', executable='sim_bridge',
        name='sim_bridge', output='screen')

    teleop = Node(
        package='robot_teleop', executable='teleop_node',
        name='teleop_node',
        parameters=[{
            'num_joints': n,
            # joint_limits_lower_deg/upper_deg NO se pasan aca: los define
            # teleop_node.py (limite visual: rango del slider + textbox).
        }],
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
