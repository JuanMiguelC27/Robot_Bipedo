# ESQUELETO de robot_simulation.
# Por ahora traduce /robot/joint_commands (JointTarget) al comando de
# posicion que usaria un controlador de Gazebo (Float64MultiArray en
# /joint_group_position_controller/commands). El encargado de simulacion
# agrega el mundo de Gazebo, los plugins y el ros_gz_bridge reales.
import rclpy
from rclpy.node import Node
from robot_interfaces.msg import JointTarget
from std_msgs.msg import Float64MultiArray


class SimBridge(Node):
    def __init__(self):
        super().__init__('sim_bridge')
        self.sub = self.create_subscription(
            JointTarget, '/robot/joint_commands', self.on_cmd, 10)
        self.pub = self.create_publisher(
            Float64MultiArray, '/joint_group_position_controller/commands', 10)
        self.get_logger().info('sim_bridge listo (esqueleto de simulacion)')

    def on_cmd(self, msg):
        out = Float64MultiArray()
        out.data = list(msg.position)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(SimBridge())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
