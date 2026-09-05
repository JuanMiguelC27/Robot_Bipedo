# Nodo puente serial -> ROS.
#
# Lee el puerto serial USB donde corre el firmware "control_pata" (que
# controla directamente el PCA9685 sin micro-ROS) y publica en ROS el
# tópico /servo_states con la posicion actual de cada servo.
#
# El firmware control_pata imprime periodicamente lineas con el formato:
#     [POS] Hip Roll=135.0deg  Hip Pitch=135.0deg  Knee=135.0deg
#
# Este nodo extrae los 3 valores y los publica como Float32MultiArray en
# el orden de los canales fisicos:
#     [0] = Hip   (canal 0)
#     [1] = Knee  (canal 1)
#     [2] = Ankle (canal 2)
#
# Uso:
#   ros2 run robot_serial_bridge serial_bridge --ros-args \
#     -p port:=/dev/ttyUSB0 -p baudrate:=115200 -p publish_rate:=2.0
import re
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

import serial


class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('publish_rate', 2.0)  # Hz

        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value
        rate = self.get_parameter('publish_rate').value

        self.port = port
        self.baud = baud

        self.pub = self.create_publisher(
            Float32MultiArray, '/servo_states', 10)

        # Patron para capturar los valores tras cada '=' en "[POS] ...=X.Xdeg"
        self._value_re = re.compile(r'=(-?[\d.]+)deg')

        self.ser = None

        # Intento abrir el puerto (y reintentara si aun no esta disponible)
        self._open_serial()

        self.create_timer(1.0 / rate, self.scan_and_publish)

    def _open_serial(self):
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.2)
            self.ser.reset_input_buffer()
            self.get_logger().info(
                f'serial bridgado en {self.port} @ {self.baud}')
        except (serial.SerialException, OSError) as e:
            self.ser = None

    def scan_and_publish(self):
        if self.ser is None:
            # Reintentar conectar en caso de que el puerto no este disponible
            self._open_serial()
            if self.ser is None:
                self.get_logger().warn(
                    f'Aun no se puede abrir {self.port}. Verifica que el '
                    f'firmware control_pata este corriendo y que ningun otro '
                    f'proceso use el puerto (pkill -f micro_ros_agent).',
                    throttle_duration_sec=5.0)
            return
        try:
            line = self.ser.readline()
        except (serial.SerialException, OSError) as e:
            self.get_logger().warn(f'Error de lectura serial: {e}')
            self.ser = None
            return

        if not line:
            return

        text = line.decode(errors='ignore')
        if '[POS]' not in text:
            return

        values = self._value_re.findall(text)
        # Tomamos los primeros 3 valores numericos encontrados
        values = values[:3]
        if len(values) < 3:
            self.get_logger().warn(
                f'Linea [POS] incompleta, se ignoran {len(values)} valores: {text.strip()}')
            return

        try:
            angles = [float(v) for v in values]
        except ValueError:
            self.get_logger().warn(f'Linea no parseable: {text.strip()}')
            return

        msg = Float32MultiArray()
        msg.data = angles   # [Hip, Knee, Ankle]
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(SerialBridge())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
