"""Puente ROS 2 Humble para DOS ESP32 independientes.

Se ejecuta en dos instancias:
  device_type=bldc   -> /robot/bldc_command   -> '-80.000\\n'
  device_type=servos -> /robot/servos_command -> 'a10.000\\n' y 'b-10.000\\n'

Por defecto SIEMPRE es simulacion (hardware_enabled=False).
No recalibra, no confirma el cero ni libera motores en E-stop.

IMPORTANTE: requiere que cada ESP32 tenga firmware para su protocolo.
No garantiza sincronizacion temporal entre las dos placas.
"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray, String
import serial


class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')

        self.declare_parameter('device_type', 'bldc')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('hardware_enabled', False)
        self.declare_parameter('send_rate', 50.0)

        self.device_type = str(self.get_parameter('device_type').value).lower()
        if self.device_type not in ('bldc', 'servos'):
            raise ValueError("device_type debe ser 'bldc' o 'servos'")

        self.port = str(self.get_parameter('port').value)
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.hardware_enabled = bool(self.get_parameter('hardware_enabled').value)
        send_rate = float(self.get_parameter('send_rate').value)
        if not 0.0 < send_rate <= 100.0 or not math.isfinite(send_rate):
            raise ValueError('send_rate debe estar entre 0 y 100 Hz')

        self.ser = None
        self.estop = False
        self.pending = None
        self.reference_stored = 'unknown'
        self._reference_cero_fragment = ''
        self.reference_validated = 'unknown'
        self.reference_connected = False
        self.reference_can = 'unknown'
        self.reference_fault = 'unknown'
        self.reference_zero = 'unknown'
        self.reference_position = 'unknown'
        self.latest_measurement = None
        self.last_measurement_ns = None
        self._rx_buffer = bytearray()
        self.bldc_telemetry_requested = False
        self.bldc_telemetry_confirmed = False
        self._invalid_frames = 0

        input_topic = ('/robot/bldc_command' if self.device_type == 'bldc'
                       else '/robot/servos_command')
        state_topic = ('/robot/bldc_state' if self.device_type == 'bldc'
                       else '/robot/servos_state')
        status_topic = ('/robot/bldc_serial_status' if self.device_type == 'bldc'
                        else '/robot/servos_serial_status')

        self.create_subscription(
            Float32MultiArray, input_topic, self.on_command, 10
        )
        self.create_subscription(Bool, '/robot/e_stop', self.on_estop, 10)
        self.state_pub = self.create_publisher(Float32MultiArray, state_topic, 10)
        self.status_pub = self.create_publisher(String, status_topic, 10)
        # Telemetría diagnóstica, independiente de la calibración articular.
        self.servo_raw_pub = None
        self.servo_flags_pub = None
        if self.device_type == 'servos':
            self.servo_raw_pub = self.create_publisher(
                Float32MultiArray, '/robot/servos_encoder_raw', 10)
            self.servo_flags_pub = self.create_publisher(
                Float32MultiArray, '/robot/servos_encoder_flags', 10)
        self.timer = self.create_timer(1.0 / send_rate, self.update)
        self.reference_pub = None
        if self.device_type == 'bldc':
            self.reference_pub = self.create_publisher(
                String, '/robot/bldc_reference_status', 10
            )
            self.create_timer(1.0, self.publish_reference_status)

        if self.hardware_enabled:
            try:
                # Igual que banco_pata: evita activar DTR/RTS al abrir.
                port = serial.Serial(baudrate=self.baudrate,
                                     timeout=0, write_timeout=0.1)
                port.port = self.port
                port.dtr = False
                port.rts = False
                port.open()
                self.ser = port
                self.reference_connected = True
                # No enviar p1 todavía: esperar una trama @R completa,
                # que confirma que el firmware terminó su arranque.
                self.get_logger().info(
                    f'{self.device_type}: puerto abierto {self.port} @ {self.baudrate}'
                )
            except (serial.SerialException, OSError) as exc:
                self.hardware_enabled = False
                self.get_logger().error(f'No se pudo abrir {self.port}: {exc}')

        if not self.hardware_enabled:
            self.get_logger().warn(
                f'{self.device_type}: MODO SIMULACION; no se envian datos a la ESP32'
            )
        self.get_logger().info(f'{self.device_type}: escuchando {input_topic}')

    def publish_reference_status(self):
        if self.reference_pub is None:
            return
        msg = String()
        msg.data = (
            f'connected={"yes" if self.ser is not None else "no"};'
            f'stored={self.reference_stored};'
            f'validated={self.reference_validated};'
            f'can={self.reference_can};fault={self.reference_fault};'
            f'zero={self.reference_zero};position={self.reference_position}'
        )
        self.reference_pub.publish(msg)

    def parse_reference_diagnostic(self, line):
        if self.device_type != 'bldc':
            return
        lower = line.lower()
        if 'cero ' in lower and '(guardado)' not in lower and '(falta)' not in lower:
            self._reference_cero_fragment = lower
        elif self._reference_cero_fragment and ('(guardado)' in lower or '(falta)' in lower):
            lower = self._reference_cero_fragment + lower
            self._reference_cero_fragment = ''
        if 'cero ' in lower and '(guardado)' in lower:
            self.reference_stored = 'yes'
        elif 'cero ' in lower and '(falta)' in lower:
            self.reference_stored = 'no'
        elif 'calibracion blDC guardada'.lower() in lower:
            # La cabecera por sí sola no garantiza que exista un cero.
            pass
        if 'referencia validada para movimiento:' in lower:
            status = lower.split('referencia validada para movimiento:', 1)[1].strip()
            if status.startswith('si') or status.startswith('sí'):
                self.reference_validated = 'yes'
            elif status.startswith('no'):
                self.reference_validated = 'no'
        # En los firmware anteriores también puede informarse un bloqueo.
        elif 'referencia validada: no' in lower:
            self.reference_validated = 'no'
        self.publish_reference_status()

    def emit_status(self, message):
        msg = String()
        msg.data = str(message)
        self.status_pub.publish(msg)

    def on_command(self, msg):
        if self.estop:
            self.emit_status('RECHAZADO|e-stop activo')
            return

        values = list(msg.data)
        expected = 1 if self.device_type == 'bldc' else 2
        if len(values) != expected or not all(math.isfinite(v) for v in values):
            self.get_logger().warn(
                f'{self.device_type}: se esperaban {expected} valores finitos; recibido {values}'
            )
            self.emit_status('RECHAZADO|longitud o valores invalidos')
            return
        if not all(-90.0 <= angle <= 90.0 for angle in values):
            self.get_logger().warn(f'{self.device_type}: fuera de rango [-90,90]: {values}')
            self.emit_status('RECHAZADO|fuera de rango')
            return

        if self.device_type == 'bldc':
            # Control_node YA envio el offset: 0 ROS -> -90 firmware.
            self.pending = [f'{values[0]:.3f}']
        else:
            # Control_node YA envio valores centrados: 0 ROS -> 0 firmware.
            self.pending = [f'a{values[0]:.3f}', f'b{values[1]:.3f}']

    def on_estop(self, msg):
        if msg.data:
            self.estop = True
            self.pending = None
            # No mandar 'x' ni 's' automáticamente: el BLDC queda libre
            # con esos comandos y la pierna puede caer por gravedad.
            self.get_logger().error(
                f'{self.device_type}: e-stop ROS; consignas nuevas bloqueadas. '
                'No sustituye al paro de seguridad fisico.'
            )
            self.emit_status('ESTOP|consignas bloqueadas')
        else:
            self.estop = False
            self.pending = None  # exige nueva consigna explicita
            self.emit_status('REARMADO|esperando nueva consigna')

    def update(self):
        self.read_telemetry()
        if self.estop or self.pending is None:
            return

        commands = self.pending
        self.pending = None
        if self.ser is None:
            line = f'SIMULACION SERIAL [{self.device_type}]: ' + ' | '.join(commands)
            self.get_logger().info(line)
            self.emit_status(line)
            return

        try:
            for command in commands:
                self.ser.write((command + '\n').encode('ascii'))
            self.emit_status('TRANSMITIDO|' + '|'.join(commands))
        except (serial.SerialException, OSError) as exc:
            self.get_logger().error(f'{self.device_type}: fallo de escritura: {exc}')
            self.emit_status(f'ERROR_SERIAL|{exc}')
            self.pending = None
            self.close_serial()

    def read_telemetry(self):
        """Reconstruir líneas completas antes de interpretar tramas @T, @S o @R."""
        if self.ser is None:
            return
        try:
            chunk = self.ser.read(min(self.ser.in_waiting, 4096))
            if not chunk:
                return
            self._rx_buffer.extend(chunk)
            if len(self._rx_buffer) > 16384:
                # Evitar acumulación indefinida de una trama sin salto de línea.
                self._rx_buffer.clear()
                self.get_logger().warn('Buffer serial excesivo: datos descartados')
                return
            count = 0
            while b'\n' in self._rx_buffer and count < 100:
                raw, _, rest = self._rx_buffer.partition(b'\n')
                self._rx_buffer = bytearray(rest)
                count += 1
                line = raw.decode('utf-8', errors='replace').strip()
                if not line:
                    continue
                try:
                    self._process_serial_line(line)
                except (ValueError, IndexError, OverflowError) as exc:
                    self._invalid_frames += 1
                    if self._invalid_frames <= 3 or self._invalid_frames % 100 == 0:
                        self.get_logger().warn(
                            f'{self.device_type}: trama inválida ({exc}): {line[:150]}')
        except (serial.SerialException, OSError) as exc:
            self.get_logger().error(f'{self.device_type}: error leyendo serial: {exc}')
            self.emit_status(f'ERROR_SERIAL|{exc}')
            self.close_serial()

    def _process_serial_line(self, line):
        fields = line.split(',')
        tag = fields[0]
        if self.device_type == 'bldc' and tag == '@R':
            # @R,ms,stored,validated,can_fresh,can_fault,zero_rev,position_rev
            if len(fields) != 8:
                raise ValueError('longitud @R')
            self.reference_stored = 'yes' if fields[2] == '1' else 'no'
            self.reference_validated = 'yes' if fields[3] == '1' else 'no'
            self.reference_can = 'yes' if fields[4] == '1' else 'no'
            self.reference_fault = fields[5]
            self.reference_zero = fields[6]
            self.reference_position = fields[7]
            self.publish_reference_status()
            # Pedir solamente telemetría. Este comando no mueve el motor
            # y no valida ni altera su referencia angular.
            if not self.bldc_telemetry_requested and self.ser is not None:
                self.ser.write(b'p1\n')
                self.bldc_telemetry_requested = True
                self.get_logger().info(
                    'BLDC preparado (@R recibido): solicitada telemetría @T mediante p1'
                )
        elif self.device_type == 'bldc' and tag == '@T':
            # @T,ms,modo,fault,angulo_real,angulo_objetivo,...
            if len(fields) < 5:
                raise ValueError('longitud @T')
            fault = int(fields[3])
            angle = float(fields[4])
            if fault == 0 and math.isfinite(angle):
                if not self.bldc_telemetry_confirmed:
                    self.bldc_telemetry_confirmed = True
                    self.get_logger().info('BLDC: primera trama @T válida recibida')
                self.publish_measurement([angle])
        elif self.device_type == 'servos' and tag == '@S':
            # @S,ms,destA,cmdA,encA,rawA,flagsA,destB,cmdB,encB,rawB,flagsB,...
            if len(fields) < 12:
                raise ValueError('longitud @S')
            enc_a, enc_b = float(fields[4]), float(fields[9])
            raw_a, raw_b = float(fields[5]), float(fields[10])
            flags_a, flags_b = int(fields[6]), int(fields[11])
            raw_msg = Float32MultiArray()
            raw_msg.data = [raw_a if (flags_a & 2) else float('nan'),
                            raw_b if (flags_b & 2) else float('nan')]
            self.servo_raw_pub.publish(raw_msg)
            flags_msg = Float32MultiArray()
            flags_msg.data = [float(flags_a), float(flags_b)]
            self.servo_flags_pub.publish(flags_msg)
            # Solo datos realmente alineados; el firmware actual NO activa bit 5.
            def calibrated(f, angle):
                return ((f & 34) == 34 and (f & 4) != 0
                        and (f & 24) == 0 and math.isfinite(angle))
            a = enc_a if calibrated(flags_a, enc_a) else float('nan')
            b = enc_b if calibrated(flags_b, enc_b) else float('nan')
            if math.isfinite(a) or math.isfinite(b):
                # NaN en una articulación no oculta la otra: teleop filtra NaN.
                self.publish_measurement([a, b])
        elif not tag.startswith('@'):
            self.parse_reference_diagnostic(line)
            self.get_logger().info(f'ESP32 {self.device_type}: {line}')
            self.emit_status('ESP32|' + line)

    def publish_measurement(self, values):
        msg = Float32MultiArray()
        msg.data = [float(v) for v in values]
        self.state_pub.publish(msg)

    def close_serial(self):
        self._rx_buffer.clear()
        self.bldc_telemetry_requested = False
        self.bldc_telemetry_confirmed = False
        self.reference_connected = False
        self.reference_stored = 'unknown'
        self.reference_validated = 'unknown'
        self.publish_reference_status()
        if self.ser is not None:
            try:
                self.ser.close()
            except (serial.SerialException, OSError):
                pass
            self.ser = None

    def destroy_node(self):
        self.close_serial()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SerialBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
