# Nodo inyector de prueba: publica /robot/command periodicamente para
# evidenciar que el flujo completo (command -> joint_targets -> control ->
# joint_states + joint_commands -> sim_bridge) funciona, sin necesidad de GUI.
import rclpy
from rclpy.node import Node
from robot_interfaces.msg import RobotCommand


class TestInjector(Node):
    def __init__(self):
        super().__init__('test_injector')
        self.declare_parameter('num_joints', 3)
        self.n = self.get_parameter('num_joints').value
        self.pub = self.create_publisher(RobotCommand, '/robot/command', 10)
        self.timer = self.create_timer(0.2, self.tick)
        self.get_logger().info('test_injector listo (publica /robot/command)')

    def tick(self):
        msg = RobotCommand()
        msg.mode = 1
        # Consigna fija de ejemplo (la cambia el encargado o el teleop real).
        msg.position = [0.1, 0.2, 0.3][:self.n] + [0.0] * max(0, self.n - 3)
        msg.velocity = [0.0] * self.n
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(TestInjector())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
