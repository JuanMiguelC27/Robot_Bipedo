import rclpy
from rclpy.node import Node
from robot_interfaces.msg import JointTarget, JointState

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.sub = self.create_subscription(
            JointTarget, '/robot/joint_target', self.on_target, 10)
        self.pub = self.create_publisher(JointState, '/robot/joint_state', 10)
        self.timer = self.create_timer(0.1, self.publish_state)
        self.get_logger().info('control_node iniciado (bajo nivel)')

    def on_target(self, msg):
        # Aquí irá el PID, equilibrio e integración IMU.
        self._target = msg

    def publish_state(self):
        state = JointState()
        state.position = [0.0]*6
        state.velocity = [0.0]*6
        state.effort = [0.0]*6
        state.battery = 100.0
        self.pub.publish(state)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ControlNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    