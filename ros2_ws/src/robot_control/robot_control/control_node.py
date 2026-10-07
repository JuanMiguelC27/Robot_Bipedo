# ============================================================
# NODO DE CONTROL (CAPA BAJA)
# ============================================================
#
# MODO VERIFICACION:
#   Recibe la consigna desde /robot/joint_targets,
#   aplica limites articulares y publica el estado tal cual
#   en /joint_states.
#
#   Sin PID, sin motor model, sin filtros. Sirve para confirmar
#   que lo que entra por los sliders se refleja en RViz.
#
# MODO PID:
#   Cuando verify_mode=false, se puede volver al modo PID para
#   control avanzado (trayectorias, IK, etc.).
#
# Uso:
#   ros2 param set /control_node verify_mode true
#
# ============================================================


import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool

from robot_interfaces.msg import JointTarget, JointState

from sensor_msgs.msg import JointState as SensorJointState


class ControlNode(Node):

    def __init__(self):

        super().__init__('control_node')


        # ====================================================
        # CONFIGURACIÓN GENERAL
        # ====================================================

        self.declare_parameter(
            'num_joints',
            3
        )


        # Default = pierna derecha
        # (Right_Hip_Roll/Pitch + Right_Knee, ver
        # robot_description/urdf/urdf_der).
        #
        # bringup_izq.launch.py sobreescribe esto con los
        # joints Left_* al lanzar la pata izquierda.

        self.declare_parameter(
            'joint_names',
            [
                'Right_Hip_Roll_Joint',
                'Right_Hip_Pitch_Joint',
                'Right_Knee_Joint'
            ]
        )


        # ====================================================
        # LIMITES ARTICULARES
        # ====================================================
        #
        # Limite INTERNO real del motor, en RADIANES.
        #
        # Ya no lo sobreescribe bringup_*.launch.py:
        # este .py es la unica fuente de verdad.
        #
        # Es un clamp de seguridad puertas adentro:
        # el operador no lo ve ni lo toca desde la interfaz
        # (eso lo hace robot_teleop, ver
        # joint_limits_lower_deg/upper_deg en teleop_node.py,
        # que debe reflejar estos mismos valores en grados).
        #
        # Para cambiarlo:
        #
        #   editar estas listas
        #
        # o:
        #
        #   ros2 param set /control_node joint_limits_lower [...]
        #   ros2 param set /control_node joint_limits_upper [...]
        #
        # Se relee cada ciclo.
        #
        # cadera-roll (indice 0): limite real 0 a 90 grados
        # (0 a 1.5708 rad), igual que lower_deg/upper_deg en
        # teleop_node.py.
        #
        # ====================================================

        self.declare_parameter(
            'joint_limits_lower',
            [
                0.0,
                -1.5708,
                -1.5708
            ]
        )

        self.declare_parameter(
            'joint_limits_upper',
            [
                1.5708,
                1.5708,
                1.5708
            ]
        )


        # ====================================================
        # MODO DE VERIFICACIÓN
        # ====================================================
        #
        # True = clamp y publicar directo.
        #
        # False = utilizar PID para control avanzado.
        #
        # ====================================================

        self.declare_parameter(
            'verify_mode',
            True
        )


        # ====================================================
        # LECTURA INICIAL DE PARÁMETROS
        # ====================================================

        self.n = self.get_parameter(
            'num_joints'
        ).value

        self.joint_names = self.get_parameter(
            'joint_names'
        ).value

        self.lower = self.get_parameter(
            'joint_limits_lower'
        ).value

        self.upper = self.get_parameter(
            'joint_limits_upper'
        ).value

        self.verify_mode = self.get_parameter(
            'verify_mode'
        ).value


        # ====================================================
        # SUSCRIPTORES
        # ====================================================

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


        # ====================================================
        # PUBLICADORES
        # ====================================================

        # Estado interno del robot.

        self.pub = self.create_publisher(
            JointState,
            '/robot/joint_states',
            10
        )


        # Estado estándar de ROS.
        #
        # Este tópico es utilizado por robot_state_publisher
        # para generar los TF que posteriormente utiliza RViz.

        self.std_pub = self.create_publisher(
            SensorJointState,
            '/joint_states',
            10
        )


        # Comando hacia simulación / actuadores.

        self.cmd_pub = self.create_publisher(
            JointTarget,
            '/robot/joint_commands',
            10
        )


        # ====================================================
        # TEMPORIZADOR
        # ====================================================
        #
        # 0.02 s = 20 ms = 50 Hz
        #
        # Actualmente es suficiente para el modo de verificación.
        #
        # ====================================================

        self.timer = self.create_timer(
            0.02,
            self.control_loop
        )


        # ====================================================
        # VARIABLES DE ESTADO
        # ====================================================

        self.target = None

        self.e_stop = False

        self._last_clamped = [
            0.0
        ] * self.n


        # ====================================================
        # INFORMACIÓN DE INICIO
        # ====================================================

        self.get_logger().info(
            f'control_node listo | '
            f'modo={"VERIFICACION" if self.verify_mode else "PID"} | '
            f'joints={self.n}'
        )

        self.get_logger().info(
            'Ángulos publicados sin inversión de signos (URDF V9)'
        )


    # ========================================================
    # RECEPCIÓN DE CONSIGNA
    # ========================================================

    def on_target(self, msg):
        """
        Guarda la consigna recibida desde el nodo de cinemática.

        La consigna llega utilizando la convención de ángulos
        definida para el robot.

        En particular:

            Hip Pitch positivo  = la pierna va hacia adelante.
            Knee Pitch positivo = la pierna va hacia atrás
                                  (flexionar/recoger rodilla).

        El URDF V9 usa esa misma convención, así que control_loop()
        publica los ángulos sin invertir signos.
        """

        self.target = msg


    # ========================================================
    # RECEPCIÓN DE E-STOP
    # ========================================================

    def on_estop(self, msg):
        """
        Actualiza el estado del paro de emergencia.
        """

        self.e_stop = msg.data


    # ========================================================
    # BUCLE PRINCIPAL DE CONTROL
    # ========================================================

    def control_loop(self):

        # ----------------------------------------------------
        # Leer limites y modo cada ciclo para que sean
        # dinamicos.
        # ----------------------------------------------------

        self.lower = self.get_parameter(
            'joint_limits_lower'
        ).value

        self.upper = self.get_parameter(
            'joint_limits_upper'
        ).value

        self.verify_mode = self.get_parameter(
            'verify_mode'
        ).value


        # ====================================================
        # CREAR ESTADO
        # ====================================================

        state = JointState()

        state.name = self.joint_names


        # ====================================================
        # E-STOP
        # ====================================================

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


        # ====================================================
        # EXISTE UNA CONSIGNA
        # ====================================================

        if self.target is not None:

            goal = (
                list(self.target.position)
                +
                [0.0] * self.n
            )

            goal = goal[
                :self.n
            ]


            # =================================================
            # MODO VERIFICACIÓN
            # =================================================

            if self.verify_mode:

                # ------------------------------------------------
                # MODO VERIFICACION:
                # clamp y publicar directo.
                # ------------------------------------------------

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


                state.position = clamped

                state.effort = [
                    0.0
                ] * self.n


            # =================================================
            # MODO PID
            # =================================================

            else:

                # ------------------------------------------------
                # MODO PID
                # (para cuando quieras agregar cinematica inversa
                # o trayectorias mas adelante).
                # ------------------------------------------------

                from robot_control.pid_controllers import (
                    PIDController
                )

                from robot_control.motor_control import (
                    MotorController
                )


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


                correction = self.pid.update(
                    self.estimated,
                    goal,
                    0.02
                )


                command = self.motor.apply(
                    goal,
                    correction
                )


                for i in range(
                    self.n
                ):

                    self.estimated[i] = max(
                        self.lower[i],
                        min(
                            self.upper[i],
                            self.estimated[i]
                            +
                            command[i] * 0.02
                        )
                    )


                state.position = self.estimated

                state.effort = command


        else:

            # =================================================
            # SIN TARGET
            # =================================================
            #
            # Mantener ultima posicion clamped.
            # =================================================

            state.position = list(
                getattr(
                    self,
                    '_last_clamped',
                    [0.0] * self.n
                )
            )

            state.effort = [
                0.0
            ] * self.n


        # ====================================================
        # INFORMACIÓN ADICIONAL DEL ESTADO
        # ====================================================

        state.velocity = [
            0.0
        ] * self.n

        state.battery = 100.0


        # ====================================================
        # GUARDAR ÚLTIMA POSICIÓN
        # ====================================================

        self._last_clamped = list(
            state.position
        )


        # ====================================================
        # SIN INVERSIÓN DE SIGNOS
        # ====================================================
        #
        # Con el URDF V9 (urdf_completo/Pata_Pacial2URDFV9.urdf)
        # los ejes de las articulaciones ya siguen la convención
        # de ángulos del robot y de la cinemática
        # (cinematica_directa_der_izq.py):
        #
        #       +Hip Pitch  -> la pierna va hacia ADELANTE
        #       +Knee Pitch -> la pierna va hacia ATRÁS
        #
        # Por eso los ángulos se publican tal cual. (Con los
        # URDF anteriores había que invertir el Knee Pitch).
        #
        # ====================================================


        # ====================================================
        # PUBLICAR ESTADO
        # ====================================================
        #
        # /joint_states -> robot_state_publisher -> RViz
        #
        # ====================================================

        self.pub.publish(
            state
        )


        self.publish_std(
            state.position,
            state.effort
        )


        # ====================================================
        # PUBLICAR COMANDO HACIA SIMULACIÓN / ACTUADORES
        # ====================================================
        #
        # sim_bridge simplemente copia este valor hacia
        # /joint_group_position_controller/commands.
        #
        # Por eso NO es necesario modificar sim_bridge.
        #
        # ====================================================

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


    # ========================================================
    # PUBLICAR SENSOR_JOINT_STATE
    # ========================================================

    def publish_std(
        self,
        position,
        effort
    ):
        """
        Publica el estado utilizando el mensaje estándar
        de ROS.

        Este tópico:

            /joint_states

        es utilizado por robot_state_publisher para generar
        los TF que RViz utiliza para visualizar el robot.

        La posición se publica tal cual (convención del robot,
        que es la misma del URDF V9).
        """

        msg = SensorJointState()

        msg.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        msg.header.frame_id = 'Base_link'

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


# ============================================================
# FUNCIÓN PRINCIPAL
# ============================================================

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


# ============================================================
# EJECUCIÓN DIRECTA
# ============================================================

if __name__ == '__main__':

    main()
