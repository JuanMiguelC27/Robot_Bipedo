# Nodo de control (capa baja).
#
# Recibe la consigna desde /robot/joint_targets,
# aplica los límites de software y los offsets de referencia
# antes de publicar el estado hacia RViz/simulación.
#
# REFERENCIA CINEMÁTICA / TELEOPERACIÓN:
#
#   Hip Roll  = 0°
#   Hip Pitch = 0°
#   Knee      = 0°
#
# REFERENCIA DEL URDF / SIMULACIÓN:
#
#   Hip Roll:
#       q_URDF = q_cinemático - 45°
#
#   Hip Pitch:
#       q_URDF = q_cinemático
#
#   Knee:
#       q_URDF = q_cinemático
#
# REFERENCIA FÍSICA DEL ROBOT:
#
#   Hip Roll:
#       q_sim = 0°  -> q_físico = 180°
#
# La inversión del sentido del motor físico se realizará
# posteriormente en la ESP32.
#
# MODO VERIFICACIÓN:
#       recibe la consigna, aplica límites y offset,
#       y publica directamente el resultado.
#
# Sin PID, sin modelo de motor y sin filtros.


import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool

from robot_interfaces.msg import JointTarget, JointState
from sensor_msgs.msg import JointState as SensorJointState


class ControlNode(Node):

    def __init__(self):

        super().__init__('control_node')

        # =========================================================
        # CONFIGURACIÓN GENERAL
        # =========================================================

        self.declare_parameter(
            'num_joints',
            6
        )

        self.declare_parameter(
            'joint_names',
            [
                'Right_Hip_Roll_Joint',
                'Right_Hip_Pitch_Joint',
                'Right_Knee_Joint',

                'Left_Hip_Roll_Joint',
                'Left_Hip_Pitch_Joint',
                'Left_Knee_Joint'
            ]
        )

        # =========================================================
        # LÍMITES DE SOFTWARE
        #
        # Estos límites corresponden a la referencia
        # CINEMÁTICA / TELEOPERACIÓN.
        #
        # Unidades: RADIANES
        #
        # HIP ROLL:
        #       -20° <= q <= +130°
        #
        # HIP PITCH:
        #       -115° <= q <= +115°
        #
        # KNEE:
        #       -115° <= q <= +115°
        # =========================================================

        self.declare_parameter(
            'joint_limits_lower',
            [
                -1.5708,    # Right Hip Roll  = -20°
                -1.5708,    # Right Hip Pitch = -115°
                -1.5708,    # Right Knee      = -115°

                -1.5708,    # Left Hip Roll   = -20°
                -1.5708,    # Left Hip Pitch  = -115°
                -1.5708     # Left Knee       = -115°
            ]
        )

        self.declare_parameter(
            'joint_limits_upper',
            [
                1.5708,     # Right Hip Roll  = +130°
                1.5708,     # Right Hip Pitch = +115°
                1.5708,     # Right Knee      = +115°

                1.5708,     # Left Hip Roll   = +130°
                1.5708,     # Left Hip Pitch  = +115°
                1.5708      # Left Knee       = +115°
            ]
        )

        # =========================================================
        # OFFSETS
        #
        # Conversión:
        #
        #       q_URDF = q_cinemático + offset
        #
        # HIP ROLL:
        #       offset = -45°
        #
        # HIP PITCH:
        #       offset = 0°
        #
        # KNEE:
        #       offset = 0°
        #
        # Unidades: RADIANES
        # =========================================================

        self.declare_parameter(
            'joint_offsets',
            [
                -0.785398,    # Right Hip Roll  = -45°
                0.0,          # Right Hip Pitch
                0.0,          # Right Knee

                -0.785398,    # Left Hip Roll   = -45°
                0.0,          # Left Hip Pitch
                0.0           # Left Knee
            ]
        )

        # =========================================================
        # MODO DE OPERACIÓN
        # =========================================================

        self.declare_parameter(
            'verify_mode',
            True
        )

        # =========================================================
        # OBTENER PARÁMETROS
        # =========================================================

        self.n = self.get_parameter(
            'num_joints'
        ).value

        self.joint_names = list(
            self.get_parameter(
                'joint_names'
            ).value
        )

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

        self.offsets = list(
            self.get_parameter(
                'joint_offsets'
            ).value
        )

        self.verify_mode = self.get_parameter(
            'verify_mode'
        ).value

        # =========================================================
        # ROS - SUSCRIPCIONES
        # =========================================================

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

        # =========================================================
        # ROS - PUBLICADORES
        # =========================================================

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

        # =========================================================
        # TEMPORIZADOR
        # =========================================================

        self.timer = self.create_timer(
            0.02,
            self.control_loop
        )

        # =========================================================
        # VARIABLES INTERNAS
        # =========================================================

        self.target = None

        self.e_stop = False

        self._last_clamped = [
            0.0
        ] * self.n

        # =========================================================
        # INFORMACIÓN
        # =========================================================

        self.get_logger().info(
            f'control_node listo | '
            f'modo={"VERIFICACION" if self.verify_mode else "PID"} | '
            f'joints={self.n}'
        )

        self.get_logger().info(
            f'Offsets aplicados: {self.offsets}'
        )

    # =============================================================
    # RECIBIR CONSIGNA
    # =============================================================

    def on_target(self, msg):

        self.target = msg

    # =============================================================
    # E-STOP
    # =============================================================

    def on_estop(self, msg):

        self.e_stop = msg.data

    # =============================================================
    # BUCLE DE CONTROL
    # =============================================================

    def control_loop(self):

        # ---------------------------------------------------------
        # Leer parámetros dinámicamente
        # ---------------------------------------------------------

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

        self.offsets = list(
            self.get_parameter(
                'joint_offsets'
            ).value
        )

        self.verify_mode = self.get_parameter(
            'verify_mode'
        ).value

        # ---------------------------------------------------------
        # Crear estado
        # ---------------------------------------------------------

        state = JointState()

        state.name = self.joint_names

        # ---------------------------------------------------------
        # E-STOP
        # ---------------------------------------------------------

        if self.e_stop:

            state.position = [
                0.0
            ] * self.n

            state.velocity = [
                0.0
            ] * self.n

            state.effort = [
                0.0
            ] * self.n

            state.battery = 100.0

            self.pub.publish(
                state
            )

            self.publish_std(
                state.position,
                state.effort
            )

            return

        # ---------------------------------------------------------
        # EXISTE UNA CONSIGNA
        # ---------------------------------------------------------

        if self.target is not None:

            goal = (
                list(self.target.position)
                + [0.0] * self.n
            )

            goal = goal[:self.n]

            # =====================================================
            # MODO VERIFICACIÓN
            # =====================================================

            if self.verify_mode:

                # -------------------------------------------------
                # 1. LIMITAR LA CONSIGNA CINEMÁTICA
                #
                # IMPORTANTE:
                # El límite se aplica ANTES del offset.
                #
                # Ejemplo Hip Roll:
                #
                #       -20° <= q <= +130°
                # -------------------------------------------------

                clamped = [
                    max(
                        self.lower[i],
                        min(
                            self.upper[i],
                            goal[i]
                        )
                    )
                    for i in range(self.n)
                ]

                # -------------------------------------------------
                # 2. APLICAR OFFSET
                #
                # q_URDF =
                #       q_cinemático + offset
                #
                # Para Hip Roll:
                #
                #       q_URDF = q_cinemático - 45°
                # -------------------------------------------------

                command_position = [
                    clamped[i] + self.offsets[i]
                    for i in range(self.n)
                ]

                # -------------------------------------------------
                # Estado publicado hacia RViz
                # -------------------------------------------------

                state.position = command_position

                state.effort = [
                    0.0
                ] * self.n

            # =====================================================
            # MODO PID
            # =====================================================

            else:

                from robot_control.pid_controllers import PIDController
                from robot_control.motor_control import MotorController

                if not hasattr(
                    self,
                    'pid'
                ):

                    self.pid = PIDController(
                        self.n
                    )

                    self.motor = MotorController(
                        self.n
                    )

                    self.estimated = [
                        0.0
                    ] * self.n

                # -------------------------------------------------
                # 1. Limitar consigna cinemática
                # -------------------------------------------------

                clamped = [
                    max(
                        self.lower[i],
                        min(
                            self.upper[i],
                            goal[i]
                        )
                    )
                    for i in range(self.n)
                ]

                # -------------------------------------------------
                # 2. Aplicar offset
                # -------------------------------------------------

                command_position = [
                    clamped[i] + self.offsets[i]
                    for i in range(self.n)
                ]

                # -------------------------------------------------
                # PID
                # -------------------------------------------------

                correction = self.pid.update(
                    self.estimated,
                    command_position,
                    0.02
                )

                command = self.motor.apply(
                    command_position,
                    correction
                )

                # -------------------------------------------------
                # Actualizar estimación
                # -------------------------------------------------

                for i in range(self.n):

                    self.estimated[i] = max(
                        self.lower[i] + self.offsets[i],
                        min(
                            self.upper[i] + self.offsets[i],
                            self.estimated[i]
                            + command[i] * 0.02
                        )
                    )

                state.position = list(
                    self.estimated
                )

                state.effort = list(
                    command
                )

        # ---------------------------------------------------------
        # SIN CONSIGNA
        # ---------------------------------------------------------

        else:

            state.position = list(
                self._last_clamped
            )

            state.effort = [
                0.0
            ] * self.n

        # ---------------------------------------------------------
        # VELOCIDAD
        # ---------------------------------------------------------

        state.velocity = [
            0.0
        ] * self.n

        state.battery = 100.0

        # ---------------------------------------------------------
        # PUBLICAR ESTADO
        # ---------------------------------------------------------

        self.pub.publish(
            state
        )

        self.publish_std(
            state.position,
            state.effort
        )

        # ---------------------------------------------------------
        # GUARDAR ÚLTIMA POSICIÓN
        # ---------------------------------------------------------

        self._last_clamped = list(
            state.position
        )

        # ---------------------------------------------------------
        # COMANDO HACIA SIMULACIÓN
        # ---------------------------------------------------------

        cmd = JointTarget()

        cmd.position = list(
            state.position
        )

        cmd.velocity = [
            0.0
        ] * self.n

        self.cmd_pub.publish(
            cmd
        )

    # =============================================================
    # PUBLICAR SENSOR_JOINT_STATE
    # =============================================================

    def publish_std(
        self,
        position,
        effort
    ):

        msg = SensorJointState()

        msg.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        msg.header.frame_id = (
            'Base_link'
        )

        msg.name = self.joint_names

        msg.position = list(
            position
        )

        msg.velocity = [
            0.0
        ] * self.n

        msg.effort = list(
            effort
        )

        self.std_pub.publish(
            msg
        )


# ================================================================
# MAIN
# ================================================================

def main(args=None):

    rclpy.init(
        args=args
    )

    node = ControlNode()

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':

    main()
