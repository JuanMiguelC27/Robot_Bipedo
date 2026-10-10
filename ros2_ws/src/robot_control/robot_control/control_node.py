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
# HARDWARE (ESP32 real):
#   Es el ÚNICO camino hacia los motores. El teleop manda la
#   pose (radianes, ángulo cinemático) por
#   /robot/hardware_command solo al apretar "Aplicar a
#   motores". Acá se revisa el e-stop y los límites
#   articulares (los mismos de RViz); si todo está bien se
#   suma el offset de cada servo y se publica en
#   /servo_commands. Si no, se RECHAZA el comando completo
#   (no se recorta) y se avisa por /robot/hardware_status.
#
#   También convierte /servo_states (grados del servo) a
#   grados cinemáticos en /robot/hardware_state, para que el
#   teleop no necesite conocer los offsets.
#
# Uso:
#   ros2 param set /control_node verify_mode true
#
# ============================================================


import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Float32MultiArray, String

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
        # OFFSETS DE LOS SERVOS (HARDWARE)
        # ====================================================
        #
        # Grados que se suman al ángulo cinemático para
        # obtener el ángulo que entiende el firmware
        # (servo = q + offset). Es la única fuente de verdad:
        # el teleop ya no los conoce.
        #
        # El firmware (servo_params.h) recorta con sus propios
        # angleMin/angleMax en ese mismo sistema de grados.
        #
        # Hip Roll: q1 = 0 es el 0° físico del servo (sin
        # offset). Hip Pitch / Knee: q = 0 es el home, el
        # centro del servo (135° físicos).
        #
        # ====================================================

        self.declare_parameter(
            'servo_offset_deg',
            [
                0.0,      # Hip Roll
                135.0,    # Hip Pitch
                135.0     # Knee
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


        # Comando para el hardware real (solo al apretar
        # "Aplicar a motores" en el teleop).

        self.hw_cmd_sub = self.create_subscription(
            JointTarget,
            '/robot/hardware_command',
            self.on_hardware_command,
            10
        )


        # Posición real de los servos, publicada por el
        # firmware del ESP32 (grados del servo).

        self.servo_state_sub = self.create_subscription(
            Float32MultiArray,
            '/servo_states',
            self.on_servo_states,
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


        # Comando hacia el ESP32 real (grados del servo, ya
        # verificado y con offset sumado).

        self.servo_pub = self.create_publisher(
            Float32MultiArray,
            '/servo_commands',
            10
        )


        # Respuesta a cada /robot/hardware_command:
        #   "OK|<texto>"         -> se envió a los motores
        #   "RECHAZADO|<motivo>" -> no se envió nada

        self.hw_status_pub = self.create_publisher(
            String,
            '/robot/hardware_status',
            10
        )


        # Posición real de los servos en grados CINEMÁTICOS
        # (/servo_states menos el offset de cada servo).

        self.hw_state_pub = self.create_publisher(
            Float32MultiArray,
            '/robot/hardware_state',
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
    # COMANDO PARA EL HARDWARE REAL
    # ========================================================

    def on_hardware_command(self, msg):
        """
        Verifica la pose pedida para los motores reales y, si
        es válida, la publica en /servo_commands.

        msg.position llega en radianes (ángulo cinemático, la
        misma convención que /robot/command). Si algo falla
        se rechaza el comando COMPLETO: recortar movería la
        pata a una pose distinta de la que se vio en RViz.
        """

        lower = self.get_parameter('joint_limits_lower').value
        upper = self.get_parameter('joint_limits_upper').value
        offset = self.get_parameter('servo_offset_deg').value


        # ----------------------------------------------------
        # 1. E-STOP
        # ----------------------------------------------------

        if self.e_stop:

            self.reject_hardware(
                'e-stop activo; no se envía nada a los motores.'
            )

            return


        # ----------------------------------------------------
        # 2. TAMAÑO Y VALORES NUMÉRICOS
        # ----------------------------------------------------

        q = list(msg.position)

        if len(q) != self.n:

            self.reject_hardware(
                f'se esperaban {self.n} ángulos y llegaron {len(q)}.'
            )

            return

        if not all(math.isfinite(v) for v in q):

            self.reject_hardware(
                'hay ángulos que no son números válidos (NaN/inf).'
            )

            return


        # ----------------------------------------------------
        # 3. LÍMITES ARTICULARES (los mismos que RViz)
        #
        # Tolerancia mínima para no rechazar soluciones de IK
        # que quedan en el borde por redondeo.
        # ----------------------------------------------------

        tol = 1e-6

        fuera = [
            f'{self.joint_names[i]}={math.degrees(q[i]):.1f}° '
            f'(rango {math.degrees(lower[i]):.1f}° a '
            f'{math.degrees(upper[i]):.1f}°)'
            for i in range(self.n)
            if not (lower[i] - tol <= q[i] <= upper[i] + tol)
        ]

        if fuera:

            self.reject_hardware(
                'fuera de límites: ' + ', '.join(fuera)
            )

            return


        # ----------------------------------------------------
        # 4. CONVERTIR A GRADOS DEL SERVO Y ENVIAR
        # ----------------------------------------------------

        q_deg = [
            math.degrees(max(lower[i], min(upper[i], q[i])))
            for i in range(self.n)
        ]

        servo_msg = Float32MultiArray()

        servo_msg.data = [
            float(q_deg[i] + offset[i])
            for i in range(self.n)
        ]

        self.servo_pub.publish(
            servo_msg
        )

        texto = ', '.join(f'{v:.1f}°' for v in q_deg)

        self.get_logger().info(
            f'Hardware: enviado q=[{texto}] -> '
            f'servo={[round(v, 1) for v in servo_msg.data]}'
        )

        status = String()
        status.data = f'OK|Enviado a los motores: [{texto}]'
        self.hw_status_pub.publish(status)


    def reject_hardware(self, motivo):
        """
        Avisa que un /robot/hardware_command no se envió.
        """

        self.get_logger().warn(
            f'Hardware: comando RECHAZADO, {motivo}'
        )

        status = String()
        status.data = f'RECHAZADO|{motivo}'
        self.hw_status_pub.publish(status)


    # ========================================================
    # POSICIÓN REAL DE LOS SERVOS
    # ========================================================

    def on_servo_states(self, msg):
        """
        Convierte /servo_states (grados del servo) a grados
        cinemáticos restando el offset de cada servo, y lo
        publica en /robot/hardware_state.
        """

        offset = self.get_parameter('servo_offset_deg').value

        n = min(self.n, len(msg.data), len(offset))

        out = Float32MultiArray()

        out.data = [
            float(msg.data[i] - offset[i])
            for i in range(n)
        ]

        self.hw_state_pub.publish(
            out
        )


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
