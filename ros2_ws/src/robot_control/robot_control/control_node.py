
import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Float32MultiArray, String
from sensor_msgs.msg import JointState as SensorJointState
from robot_interfaces.msg import JointTarget, JointState


class ControlNode(Node):

    def __init__(self):
        super().__init__('control_node')

        # ==============================================
        # CONFIGURACION GENERAL
        # ==============================================

        self.declare_parameter('num_joints', 3)

        self.declare_parameter(
            'joint_names',
            [
                'Right_Hip_Roll_Joint',
                'Right_Hip_Pitch_Joint',
                'Right_Knee_Joint'
            ]
        )

        # Limites cinematicos en radianes.
        # Deben coincidir con los limites de la GUI
        # y estar dentro del recorrido mecanico real.
        self.declare_parameter(
            'joint_limits_lower',
            [0.0, -1.5708, -1.5708]
        )

        self.declare_parameter(
            'joint_limits_upper',
            [1.5708, 1.5708, 1.5708]
        )

        # ==============================================
        # OFFSETS DEL HARDWARE
        # ==============================================
        #
        # Home ROS: [0, 0, 0] grados
        #
        # Home /servo_commands: [-90, 90, 90]
        #
        # serial_bridge convierte a:
        #   BLDC       -> -90
        #   Hip Pitch  -> a0
        #   Knee Pitch -> b0
        #
        # Los offsets son referencias del protocolo,
        # no necesariamente angulos mecanicos absolutos.
        # ==============================================

        self.declare_parameter(
            'servo_offset_deg',
            [
                -90.0,  # Hip Roll - BLDC: q_ROS=0 -> -90 en firmware
                90.0,   # Hip Pitch
                90.0    # Knee Pitch
            ]
        )

        self.declare_parameter('verify_mode', True)

        self.n = int(
            self.get_parameter('num_joints').value
        )

        self.joint_names = list(
            self.get_parameter('joint_names').value
        )

        self.lower = list(
            self.get_parameter('joint_limits_lower').value
        )

        self.upper = list(
            self.get_parameter('joint_limits_upper').value
        )

        self.verify_mode = self.get_parameter(
            'verify_mode'
        ).value

        self.target = None
        self.e_stop = False
        self._last_clamped = [0.0] * self.n

        # ==============================================
        # SUSCRIPTORES
        # ==============================================

        self.sub = self.create_subscription(
            JointTarget,
            '/robot/joint_targets',
            self.on_target,
            10
        )

        self.estop_sub = self.create_subscription(
            Bool,
            '/robot/e_stop',
            self.on_estop,
            10
        )

        self.hw_cmd_sub = self.create_subscription(
            JointTarget,
            '/robot/hardware_command',
            self.on_hardware_command,
            10
        )

        self.servo_state_sub = self.create_subscription(
            Float32MultiArray,
            '/servo_states',
            self.on_servo_states,
            10
        )

        # ==============================================
        # PUBLICADORES
        # ==============================================

        self.pub = self.create_publisher(
            JointState,
            '/robot/joint_states',
            10
        )

        self.std_pub = self.create_publisher(
            SensorJointState,
            '/joint_states',
            10
        )

        self.cmd_pub = self.create_publisher(
            JointTarget,
            '/robot/joint_commands',
            10
        )

        self.servo_pub = self.create_publisher(
            Float32MultiArray,
            '/servo_commands',
            10
        )

        self.hw_status_pub = self.create_publisher(
            String,
            '/robot/hardware_status',
            10
        )

        self.hw_state_pub = self.create_publisher(
            Float32MultiArray,
            '/robot/hardware_state',
            10
        )

        # Control a 50 Hz
        self.timer = self.create_timer(
            0.02,
            self.control_loop
        )

        self.get_logger().info(
            'Control iniciado | '
            f'Joints={self.n} | '
            'Offsets=[-90, 90, 90]'
        )

    # ==============================================
    # CONSIGNAS CINEMATICAS
    # ==============================================

    def on_target(self, msg):
        self.target = msg

    # ==============================================
    # PARO DE EMERGENCIA
    # ==============================================

    def on_estop(self, msg):
        self.e_stop = bool(msg.data)

        if self.e_stop:
            self.get_logger().warn(
                'E-stop activo: comandos de hardware bloqueados'
            )

    # ==============================================
    # ENVIO A HARDWARE
    # ==============================================

    def on_hardware_command(self, msg):

        if self.e_stop:
            self.reject_hardware(
                'E-stop activo'
            )
            return

        q = list(msg.position)

        if len(q) != self.n:
            self.reject_hardware(
                f'Se esperaban {self.n} articulaciones'
            )
            return

        if not all(math.isfinite(v) for v in q):
            self.reject_hardware(
                'Angulos invalidos'
            )
            return

        lower = list(
            self.get_parameter(
                'joint_limits_lower'
            ).value
        )

        upper = list(
            self.get_parameter(
                'joint_limits_upper'
            ).value
        )

        offsets = list(
            self.get_parameter(
                'servo_offset_deg'
            ).value
        )

        if not (
            len(lower) == len(upper)
            == len(offsets) == self.n
        ):
            self.reject_hardware(
                'Configuracion de limites u offsets invalida'
            )
            return

        if not all(
            math.isfinite(v)
            for v in lower + upper + offsets
        ):
            self.reject_hardware(
                'Parametros no finitos'
            )
            return

        tol = 1e-6

        for i in range(self.n):

            if not (
                lower[i] - tol
                <= q[i]
                <= upper[i] + tol
            ):
                self.reject_hardware(
                    f'{self.joint_names[i]} fuera de limites'
                )
                return

        # Convertir radianes cinematicos a grados.
        q_deg = [
            math.degrees(
                max(lower[i], min(upper[i], q[i]))
            )
            for i in range(self.n)
        ]

        # Aplicar los offsets del protocolo.
        physical = [
            float(q_deg[i] + offsets[i])
            for i in range(self.n)
        ]

        # Verificar el rango que acepta el puente serial.
        # BLDC: -90 a 90
        # Servos: angulo comunicado de 0 a 180; el puente resta 90 para a/b
        if self.n == 3:
            if not (
                -90.0 <= physical[0] <= 90.0
                and 0.0 <= physical[1] <= 180.0
                and 0.0 <= physical[2] <= 180.0
            ):
                self.reject_hardware(
                    'Objetivo fuera del rango del firmware'
                )
                return

        out = Float32MultiArray()
        out.data = physical

        self.servo_pub.publish(out)

        texto = ', '.join(
            f'{v:.1f}' for v in q_deg
        )

        self.get_logger().info(
            f'Hardware q=[{texto}] -> {physical}'
        )

        status = String()
        status.data = (
            f'OK|Enviado a los motores: [{texto}]'
        )
        self.hw_status_pub.publish(status)

    def reject_hardware(self, motivo):

        self.get_logger().warn(
            f'Hardware rechazado: {motivo}'
        )

        status = String()
        status.data = f'RECHAZADO|{motivo}'

        self.hw_status_pub.publish(status)

    # ==============================================
    # REALIMENTACION DEL HARDWARE
    # ==============================================

    def on_servo_states(self, msg):

        offsets = list(
            self.get_parameter(
                'servo_offset_deg'
            ).value
        )

        if len(msg.data) != self.n:
            return

        if len(offsets) != self.n:
            return

        if not all(math.isfinite(v) for v in msg.data):
            return

        out = Float32MultiArray()

        out.data = [
            float(msg.data[i] - offsets[i])
            for i in range(self.n)
        ]

        self.hw_state_pub.publish(out)

    # ==============================================
    # BUCLE DE CONTROL
    # ==============================================

    def control_loop(self):

        self.lower = list(
            self.get_parameter(
                'joint_limits_lower'
            ).value
        )

        self.upper = list(
            self.get_parameter(
                'joint_limits_upper'
            ).value
        )

        self.verify_mode = self.get_parameter(
            'verify_mode'
        ).value

        if (
            len(self.lower) != self.n
            or len(self.upper) != self.n
        ):
            return

        state = JointState()
        state.name = list(self.joint_names)

        if self.e_stop:

            state.position = [0.0] * self.n
            state.velocity = [0.0] * self.n
            state.effort = [0.0] * self.n
            state.battery = 100.0

            self.pub.publish(state)
            self.publish_std(
                state.position,
                state.effort
            )
            return

        if self.target is not None:

            goal = (
                list(self.target.position)
                + [0.0] * self.n
            )[:self.n]

            if not all(math.isfinite(v) for v in goal):
                self.get_logger().warn(
                    'Consigna de simulacion invalida',
                    throttle_duration_sec=5.0
                )
                return

            if self.verify_mode:

                clamped = [
                    max(
                        self.lower[i],
                        min(self.upper[i], goal[i])
                    )
                    for i in range(self.n)
                ]

                state.position = clamped
                state.effort = [0.0] * self.n

            else:

                from robot_control.pid_controllers import (
                    PIDController
                )

                from robot_control.motor_control import (
                    MotorController
                )

                if not hasattr(self, 'pid'):

                    self.pid = PIDController(self.n)
                    self.motor = MotorController(self.n)
                    self.estimated = [0.0] * self.n

                correction = self.pid.update(
                    self.estimated,
                    goal,
                    0.02
                )

                command = self.motor.apply(
                    goal,
                    correction
                )

                for i in range(self.n):

                    self.estimated[i] = max(
                        self.lower[i],
                        min(
                            self.upper[i],
                            self.estimated[i]
                            + command[i] * 0.02
                        )
                    )

                state.position = list(self.estimated)
                state.effort = list(command)

        else:

            state.position = list(
                self._last_clamped
            )

            state.effort = [0.0] * self.n

        state.velocity = [0.0] * self.n
        state.battery = 100.0

        self._last_clamped = list(
            state.position
        )

        self.pub.publish(state)

        self.publish_std(
            state.position,
            state.effort
        )

        cmd = JointTarget()
        cmd.position = list(state.position)
        cmd.velocity = [0.0] * self.n

        self.cmd_pub.publish(cmd)

    # ==============================================
    # PUBLICAR ESTADO PARA RVIZ
    # ==============================================

    def publish_std(self, position, effort):

        msg = SensorJointState()

        msg.header.stamp = (
            self.get_clock().now().to_msg()
        )

        msg.header.frame_id = 'Base_link'
        msg.name = list(self.joint_names)
        msg.position = list(position)
        msg.velocity = [0.0] * self.n
        msg.effort = list(effort)

        self.std_pub.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = ControlNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
