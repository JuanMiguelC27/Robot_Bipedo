# Nodo de control (capa baja).
# MODO VERIFICACION: recibe la consigna desde /robot/joint_targets,
# aplica limites articulares y publica el estado tal cual en /joint_states.
# Sin PID, sin motor model, sin filtros. Sirve para confirmar que lo que
# entra por los sliders se refleja identico en RViz.
#
# Uso:
#   ros2 param set /control_node verify_mode true
#
# Cuando verify_mode=false, se puede volver al modo PID para control
# avanzado (trayectorias, IK, etc.).
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from robot_interfaces.msg import JointTarget, JointState
from sensor_msgs.msg import JointState as SensorJointState


class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.declare_parameter('num_joints', 3)
        self.declare_parameter('joint_names',
                               ['Hip_Joint', 'Knee_Joint', 'Ankle_Joint'])
        # Limites en RADIANES. Cambiables en vivo con ros2 param set.
        self.declare_parameter('joint_limits_lower',
                               [-1.5708, -1.5708, -1.5708])
        self.declare_parameter('joint_limits_upper',
                               [1.5708, 1.5708, 1.5708])
        # Modo verificacion: True = clamp y publicar directo.
        self.declare_parameter('verify_mode', True)

        self.n = self.get_parameter('num_joints').value
        self.joint_names = self.get_parameter('joint_names').value
        self.lower = self.get_parameter('joint_limits_lower').value
        self.upper = self.get_parameter('joint_limits_upper').value
        self.verify_mode = self.get_parameter('verify_mode').value

        self.sub = self.create_subscription(
            JointTarget, '/robot/joint_targets', self.on_target, 10)
        self.pub = self.create_publisher(
            JointState, '/robot/joint_states', 10)
        self.std_pub = self.create_publisher(
            SensorJointState, '/joint_states', 10)
        self.cmd_pub = self.create_publisher(
            JointTarget, '/robot/joint_commands', 10)
        self.estop_sub = self.create_subscription(
            Bool, '/robot/e_stop', self.on_estop, 10)
        self.timer = self.create_timer(0.02, self.control_loop)

        self.target = None
        self.e_stop = False
        self.get_logger().info(
            f'control_node listo | modo={"VERIFICACION" if self.verify_mode else "PID"} | '
            f'joints={self.n}')

    def on_target(self, msg):
        self.target = msg

    def on_estop(self, msg):
        self.e_stop = msg.data

    def control_loop(self):
        # Leer limites y modo cada ciclo para que sean dinamicos.
        self.lower = self.get_parameter('joint_limits_lower').value
        self.upper = self.get_parameter('joint_limits_upper').value
        self.verify_mode = self.get_parameter('verify_mode').value

        state = JointState()
        state.name = self.joint_names

        if self.e_stop:
            state.position = [0.0] * self.n
            state.velocity = [0.0] * self.n
            state.effort = [0.0] * self.n
            state.battery = 100.0
            self.pub.publish(state)
            self.publish_std(state.position, state.effort)
            return

        if self.target is not None:
            goal = list(self.target.position) + [0.0] * self.n
            goal = goal[:self.n]

            if self.verify_mode:
                # MODO VERIFICACION: clamp y publicar directo.
                clamped = [max(self.lower[i], min(self.upper[i], goal[i]))
                           for i in range(self.n)]
                state.position = clamped
                state.effort = [0.0] * self.n
            else:
                # MODO PID (para cuando quieras agregar cinematica inversa
                # o trayectorias mas adelante).
                from robot_control.pid_controllers import PIDController
                from robot_control.motor_control import MotorController
                if not hasattr(self, 'pid'):
                    self.pid = PIDController(self.n)
                    self.motor = MotorController(self.n)
                    self.estimated = [0.0] * self.n
                correction = self.pid.update(self.estimated, goal, 0.02)
                command = self.motor.apply(goal, correction)
                for i in range(self.n):
                    self.estimated[i] = max(self.lower[i], min(self.upper[i],
                                                               self.estimated[i] + command[i] * 0.02))
                state.position = self.estimated
                state.effort = command
        else:
            # Sin target: mantener ultima posicion clamped.
            state.position = list(getattr(self, '_last_clamped', [0.0] * self.n))
            state.effort = [0.0] * self.n

        state.velocity = [0.0] * self.n
        state.battery = 100.0
        self.pub.publish(state)
        self.publish_std(state.position, state.effort)

        # Guardar ultima posicion clamped para cuando no haya target.
        self._last_clamped = list(state.position)

        # Publicar comando hacia simulacion (clampado).
        cmd = JointTarget()
        cmd.position = list(state.position)
        cmd.velocity = [0.0] * self.n
        self.cmd_pub.publish(cmd)

    def publish_std(self, position, effort):
        msg = SensorJointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'Base_link'
        msg.name = self.joint_names
        msg.position = list(position)
        msg.velocity = [0.0] * self.n
        msg.effort = list(effort)
        self.std_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ControlNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
