# Nodo de cinemática (capa alta del control).
# Por ahora reenvía la orden del operador como consigna articular.
# Más adelante aquí calculas IK, trayectoria y coordinación de patas.
import rclpy
from rclpy.node import Node
from robot_interfaces.msg import RobotCommand, JointTarget


class KinematicsNode(Node):
    def __init__(self):
        super().__init__('kinematics_node')
        # n = cantidad de juntas: 3 (una pata) o 6 (robot completo).
        self.declare_parameter('num_joints', 3)
        self.n = self.get_parameter('num_joints').value

        # Orden que llega del teleop.
        self.sub = self.create_subscription(
            RobotCommand, '/robot/command', self.on_cmd, 10)
        # Consigna que sale hacia el control.
        self.pub = self.create_publisher(
            JointTarget, '/robot/joint_targets', 10)
        self.get_logger().info(f'kinematics_node listo con {self.n} juntas')

    def on_cmd(self, msg):
        # TODO: aquí va la IK real. Hoy copiamos position/velocity
        # y recortamos al tamaño n por si mandan más valores.
        target = JointTarget()
        target.position = list(msg.position[:self.n])
        target.velocity = list(msg.velocity[:self.n])
        self.pub.publish(target)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(KinematicsNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()