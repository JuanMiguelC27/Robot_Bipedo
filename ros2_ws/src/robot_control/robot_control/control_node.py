# Nodo de control (capa baja). Recibe la consigna de cinemática,
# la sigue con un PID y publica el estado y el comando de las juntas.
# El comando se recorta al limite de cada motor (entrada del lider).
# ESQUELETO: la logica real (equilibrio, IMU, MPC) la agrega el encargado.
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from robot_interfaces.msg import JointTarget, JointState
from sensor_msgs.msg import JointState as SensorJointState
from robot_control.pid_controllers import PIDController
from robot_control.motor_control import MotorController


class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.declare_parameter('num_joints', 3)
        # Nombres de las juntas DEBEN coincidir EXACTO con el URDF, porque
        # robot_state_publisher usa este campo para emparejar angulo->junta y
        # publicar el TF. Sin name, RViz dice "No transform".
        self.declare_parameter('joint_names',
                               ['hip_1', 'hip_2', 'knee'])
        # LIMITE POR MOTOR (coincide con el URDF de robot_description).
        # Entrada para restringir el angulo de cada articulacion.
        self.declare_parameter('joint_limits_lower', [-1.5, -1.5, -2.0])
        self.declare_parameter('joint_limits_upper', [1.5, 1.5, 0.0])
        self.n = self.get_parameter('num_joints').value
        self.joint_names = self.get_parameter('joint_names').value
        self.lower = self.get_parameter('joint_limits_lower').value
        self.upper = self.get_parameter('joint_limits_upper').value

        self.sub = self.create_subscription(
            JointTarget, '/robot/joint_targets', self.on_target, 10)
        self.pub = self.create_publisher(
            JointState, '/robot/joint_states', 10)
        # Comando hacia la simulacion (Gazebo lo lee via robot_simulation).
        self.cmd_pub = self.create_publisher(
            JointTarget, '/robot/joint_commands', 10)
        # Estado fisico estandar para robot_state_publisher (este nodo SI lo
        # entiende, porque escucha /joint_states por defecto, no /robot/*).
        self.std_pub = self.create_publisher(
            SensorJointState, '/joint_states', 10)
        self.estop_sub = self.create_subscription(
            Bool, '/robot/e_stop', self.on_estop, 10)
        self.timer = self.create_timer(0.02, self.control_loop)

        self.pid = PIDController(self.n)
        self.motor = MotorController(self.n)
        self.estimated = [0.0] * self.n
        self.target = None
        self.e_stop = False
        self.get_logger().info(f'control_node listo con {self.n} juntas')

    def on_target(self, msg):
        self.target = msg

    def on_estop(self, msg):
        self.e_stop = msg.data

    def control_loop(self):
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

        command = [0.0] * self.n
        if self.target is not None:
            goal = list(self.target.position) + [0.0] * self.n
            goal = goal[:self.n]
            correction = self.pid.update(self.estimated, goal, 0.02)
            command = self.motor.apply(goal, correction)
            for i in range(self.n):
                self.estimated[i] += command[i] * 0.02
        # SIEMPRE asignar position/effort (aunque no haya target) para que
        # el JointState estandar nunca llegue con 'position' vacio; si no,
        # robot_state_publisher lo ignora y no publica TF.
        state.position = list(self.estimated)
        state.effort = list(command)
        state.velocity = [0.0] * self.n
        state.battery = 100.0
        self.pub.publish(state)
        self.publish_std(state.position, state.effort)

        # CLAMP: recorta el comando al limite de cada motor y lo publica.
        cmd = JointTarget()
        cmd.position = [max(self.lower[i], min(self.upper[i], command[i]))
                        for i in range(self.n)]
        cmd.velocity = [0.0] * self.n
        self.cmd_pub.publish(cmd)

    def publish_std(self, position, effort):
        # Publica en /joint_states (estandar) para que robot_state_publisher
        # genere los TF. Mismos nombres y angulos que el estado interno.
        # Hay que poner header.stamp y frame_id, si no el state_publisher
        # descarta el mensaje por "viejo" y no publica TF.
        msg = SensorJointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
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
