
import math
import serial
import rclpy

from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool, String


class SerialBridge(Node):

    def __init__(self):
        super().__init__('serial_bridge')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('hardware_enabled', False)
        self.declare_parameter('send_rate', 50.0)

        port = self.get_parameter('port').value
        baudrate = int(self.get_parameter('baudrate').value)
        rate = float(self.get_parameter('send_rate').value)

        self.hardware_enabled = bool(
            self.get_parameter('hardware_enabled').value
        )

        if rate <= 0:
            raise ValueError('send_rate debe ser mayor que cero')

        self.ser = None
        self.target = None
        self.pending = False
        self.estop = False

        self.bldc_angle = None
        self.pitch_angle = None
        self.knee_angle = None

        self.bldc_time = None
        self.servo_time = None

        self.cmd_sub = self.create_subscription(
            Float32MultiArray,
            '/servo_commands',
            self.on_command,
            10
        )

        self.estop_sub = self.create_subscription(
            Bool,
            '/robot/e_stop',
            self.on_estop,
            10
        )

        self.state_pub = self.create_publisher(
            Float32MultiArray,
            '/servo_states',
            10
        )

        self.status_pub = self.create_publisher(
            String,
            '/robot/serial_status',
            10
        )

        if self.hardware_enabled:
            try:
                self.ser = serial.Serial(
                    port,
                    baudrate,
                    timeout=0,
                    write_timeout=0.1
                )
                self.get_logger().info(
                    f'ESP32 conectada en {port}'
                )
            except (serial.SerialException, OSError) as e:
                self.get_logger().error(
                    f'No se pudo abrir el puerto: {e}'
                )
                self.hardware_enabled = False

        if not self.hardware_enabled:
            self.get_logger().warn(
                'MODO PRUEBA: no se enviaran comandos a motores'
            )

        self.timer = self.create_timer(
            1.0 / rate,
            self.update
        )

    def on_command(self, msg):

        if len(msg.data) != 3:
            self.get_logger().warn(
                'Se esperaban 3 angulos articulares'
            )
            return

        values = list(msg.data)

        if not all(math.isfinite(v) for v in values):
            self.get_logger().warn(
                'Comando rechazado: valores no finitos'
            )
            return

        # /servo_commands usa grados fisicos.
        bldc, pitch, knee = values

        # Firmware BLDC: angulo firmado en grados.
        # Firmware servos: angulo fisico = 90 + comando.
        cmd_bldc = bldc
        cmd_pitch = pitch - 90.0
        cmd_knee = knee - 90.0

        if not all(
            -90.0 <= v <= 90.0
            for v in (cmd_bldc, cmd_pitch, cmd_knee)
        ):
            self.get_logger().warn(
                f'Comando fuera del rango del firmware: {values}'
            )
            return

        self.target = [
            f'{cmd_bldc:.3f}',
            f'a{cmd_pitch:.3f}',
            f'b{cmd_knee:.3f}'
        ]
        self.pending = True

    def on_estop(self, msg):

        if msg.data:
            self.estop = True
            self.pending = False
            self.target = None

            # En este firmware, x libera los motores.
            if self.ser is not None:
                try:
                    self.ser.write(b'x\n')
                except (serial.SerialException, OSError):
                    pass

            self.get_logger().error('E-STOP activado')
        else:
            # No se rearma automaticamente el movimiento.
            self.estop = False
            self.pending = False
            self.target = None

    def update(self):

        self.read_telemetry()

        if self.estop or not self.pending:
            return

        if self.target is None:
            return

        commands = self.target
        self.pending = False

        if self.ser is None:
            self.get_logger().info(
                'SIMULACION SERIAL: ' + ' | '.join(commands)
            )
            return

        try:
            for command in commands:
                self.ser.write((command + '\n').encode('ascii'))

        except (serial.SerialException, OSError) as e:
            self.get_logger().error(
                f'Error enviando comandos: {e}'
            )
            self.hardware_enabled = False
            self.ser.close()
            self.ser = None

    def read_telemetry(self):

        if self.ser is None:
            return

        try:
            # Limitar las lineas procesadas por ciclo.
            for _ in range(15):
                if self.ser.in_waiting == 0:
                    break

                line = self.ser.readline().decode(
                    'utf-8', errors='ignore'
                ).strip()

                if not line:
                    continue

                fields = line.split(',')
                now = self.get_clock().now().nanoseconds

                if fields[0] == '@T' and len(fields) >= 5:
                    angle = float(fields[4])
                    if math.isfinite(angle):
                        self.bldc_angle = angle
                        self.bldc_time = now

                elif fields[0] == '@S' and len(fields) >= 12:
                    # @S,ms,destA,cmdA,encA,rawA,flagsA,
                    #       destB,cmdB,encB,rawB,flagsB,...
                    enc_a = float(fields[4])
                    enc_b = float(fields[9])
                    flags_a = int(fields[6])
                    flags_b = int(fields[11])

                    # Bit 1: encoder responde.
                    # Bit 5: encoder alineado.
                    valid_a = (flags_a & 34) == 34
                    valid_b = (flags_b & 34) == 34

                    if (valid_a and valid_b
                            and math.isfinite(enc_a)
                            and math.isfinite(enc_b)):

                        self.pitch_angle = 90.0 + enc_a
                        self.knee_angle = 90.0 + enc_b
                        self.servo_time = now

        except (ValueError, IndexError):
            self.get_logger().warn(
                'Trama de telemetria invalida',
                throttle_duration_sec=5.0
            )
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f'Error serial: {e}')
            self.ser.close()
            self.ser = None
            return

        # Publicar solo mediciones recientes y validas.
        if None in (
            self.bldc_angle, self.pitch_angle,
            self.knee_angle, self.bldc_time, self.servo_time
        ):
            return

        now = self.get_clock().now().nanoseconds
        max_age_ns = 500_000_000

        if (now - self.bldc_time > max_age_ns or
                now - self.servo_time > max_age_ns):
            return

        msg = Float32MultiArray()
        msg.data = [
            self.bldc_angle,
            self.pitch_angle,
            self.knee_angle
        ]
        self.state_pub.publish(msg)

    def destroy_node(self):
        if self.ser is not None:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

