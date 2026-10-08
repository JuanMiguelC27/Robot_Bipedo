# ============================================================
# NODO DE CINEMÁTICA INVERSA (GRADIENTE DESCENDENTE)
# ============================================================
#
# Recibe un objetivo (x, y, z, semilla q1/q2/q3) en
# /robot/ik_grad_target y publica en /robot/ik_grad_result:
#
#   - reachable: si el método converge.
#   - position:  q1, q2, q3 [rad] resultantes (si aplica).
#
# Usa el mismo mensaje IKJacobTarget que el Jacobiano, porque
# también es iterativo y necesita semilla.
#
# ============================================================

import numpy as np

import rclpy
from rclpy.node import Node

from robot_interfaces.msg import IKJacobTarget, IKResult

from robot_kinematics.kinem_invers_leg_gradiente_descendente import (
    cinematica_inversa_pata_grad,
    joint_limit_warnings,
)


class IKGradNode(Node):

    def __init__(self):
        super().__init__('ik_grad_node')

        self.declare_parameter('leg_side', 'left')
        self.leg_side = (
            self.get_parameter('leg_side')
            .get_parameter_value()
            .string_value
            .lower()
        )

        self.target_sub = self.create_subscription(
            IKJacobTarget, '/robot/ik_grad_target', self.on_target, 10)

        self.result_pub = self.create_publisher(
            IKResult, '/robot/ik_grad_result', 10)

        self.get_logger().info(
            f'ik_grad_node listo (pierna {self.leg_side})')

    def on_target(self, msg):
        q1, q2, q3, alcanzable = cinematica_inversa_pata_grad(
            msg.x, msg.y, msg.z,
            msg.q1_seed, msg.q2_seed, msg.q3_seed)

        result = IKResult()
        result.reachable = bool(alcanzable)

        if alcanzable:
            result.position = [float(q1), float(q2), float(q3)]

            q_deg = np.degrees([q1, q2, q3])
            for warning in joint_limit_warnings(*q_deg):
                self.get_logger().warn(warning)
        else:
            result.position = []
            self.get_logger().warn(
                f'Objetivo ({msg.x:.1f}, {msg.y:.1f}, {msg.z:.1f}) '
                f'no convergió desde la semilla '
                f'({msg.q1_seed:.1f}, {msg.q2_seed:.1f}, '
                f'{msg.q3_seed:.1f})°.'
            )

        self.result_pub.publish(result)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(IKGradNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
