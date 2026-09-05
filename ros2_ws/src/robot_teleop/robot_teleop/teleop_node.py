# Interfaz de barras deslizantes para teleoperar la pata.
# Cada slider es una junta: al moverlo publica /robot/command.
# El operador ve GRADOS; internamente todo se convierte a RADIANES.
#
# Como ROS y tkinter NO pueden correr en el mismo hilo, rclpy.spin va en un
# hilo aparte (daemon) y la GUI corre en el principal con root.mainloop().
# Los sliders se refrescan desde el hilo de la GUI con root.after(), no desde
# el callback de ROS, para que no se trabe (ese era el retardo).
import math
import threading
import rclpy
from rclpy.node import Node
from robot_interfaces.msg import RobotCommand, JointState
from std_msgs.msg import Float32MultiArray
import tkinter as tk

from robot_teleop.forward_kinematics import end_effector_matrix


class GuiNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.declare_parameter('num_joints', 3)
        # Servos de 270° -> rango util -135° a +135° por slider.
        self.declare_parameter('joint_limits_lower_deg', [-120.0, -105.0, -105.0])
        self.declare_parameter('joint_limits_upper_deg', [120.0, 105.0, 105.0])
        # Debe coincidir con home_angle[] en firmware/esp32_servos/src/main.cpp.
        # slider = 0 -> el servo queda exactamente en este angulo fisico.
        self.declare_parameter('home_angle_deg', [180.0, 135.0, 135.0])
        self.n = self.get_parameter('num_joints').value
        self.lower_deg = self.get_parameter('joint_limits_lower_deg').value
        self.upper_deg = self.get_parameter('joint_limits_upper_deg').value
        self.home_angle_deg = self.get_parameter('home_angle_deg').value
        self.declare_parameter('dh_link_lengths', [1.0] * 5)
        self.dh_link_lengths = self.get_parameter('dh_link_lengths').value

        self.cmd_pub = self.create_publisher(RobotCommand, '/robot/command', 10)
        self.servo_pub = self.create_publisher(
            Float32MultiArray, '/servo_commands', 10)
        self.create_subscription(JointState, '/robot/joint_states', self.on_state, 10)

        self.dragging = False
        self.updating_sliders = False
        self.target_deg = [0.0] * self.n   # consigna en grados del operador
        self.latest_state_deg = [0.0] * self.n  # estado real (no se usa para sliders)
        self.root = tk.Tk()
        self.root.title('Teleop pata bipedo (sliders en grados)')
        self.sliders = []
        self.value_labels = []
        for i in range(self.n):
            tk.Label(self.root, text=f'junta {i}').grid(row=i, column=0)
            s = tk.Scale(self.root, from_=self.lower_deg[i], to=self.upper_deg[i],
                         resolution=0.5, orient=tk.HORIZONTAL, length=320,
                         command=lambda v, idx=i: self.on_slide(idx, v))
            s.bind('<ButtonPress-1>', lambda e, idx=i: self.set_drag(True, idx))
            s.bind('<ButtonRelease-1>', lambda e, idx=i: self.set_drag(False, idx))
            s.grid(row=i, column=1)
            self.sliders.append(s)
            lbl = tk.Label(self.root, text='0.0°')
            lbl.grid(row=i, column=2)
            self.value_labels.append(lbl)

        input_frame = tk.LabelFrame(
            self.root,
            text='Ingresar angulos manualmente (grados)',
            padx=10,
            pady=8,
        )
        input_frame.grid(row=self.n, column=0, columnspan=3,
                         padx=10, pady=(10, 0), sticky='ew')
        self.angle_vars = []
        self.angle_entries = []
        for i in range(self.n):
            tk.Label(input_frame, text=f'Junta {i}:').grid(
                row=0, column=i * 2, padx=(4, 2), pady=2)
            angle_var = tk.StringVar(value='0.0')
            entry = tk.Entry(input_frame, textvariable=angle_var, width=9,
                             justify='right')
            entry.grid(row=0, column=i * 2 + 1, padx=(0, 8), pady=2)
            entry.bind('<Return>', lambda event: self.apply_entry_angles())
            self.angle_vars.append(angle_var)
            self.angle_entries.append(entry)

        tk.Button(input_frame, text='Aplicar angulos',
                  command=self.apply_entry_angles).grid(
                      row=0, column=self.n * 2, padx=(8, 4), pady=2)
        self.input_status = tk.Label(input_frame, text='', anchor='w')
        self.input_status.grid(row=1, column=0, columnspan=self.n * 2 + 1,
                               sticky='w', padx=4, pady=(4, 0))

        matrix_frame = tk.LabelFrame(
            self.root,
            text='Matriz resultante de cinematica directa (0A4)',
            padx=10,
            pady=8,
        )
        matrix_frame.grid(row=self.n + 1, column=0, columnspan=3,
                          padx=10, pady=12, sticky='ew')
        self.matrix_labels = []
        for row in range(4):
            label_row = []
            for column in range(4):
                label = tk.Label(matrix_frame, width=12, anchor='e',
                                 font=('TkFixedFont', 10), relief='sunken',
                                 padx=4, pady=3)
                label.grid(row=row, column=column, padx=2, pady=2)
                label_row.append(label)
            self.matrix_labels.append(label_row)

        tk.Label(
            self.root,
            text='Rotacion: columnas 1-3 | Posicion: ultima columna',
            fg='gray35',
        ).grid(row=self.n + 2, column=0, columnspan=3, pady=(0, 8))

        self.update_kinematics_matrix()

        self.root.after(50, self.refresh)

    def set_drag(self, val, idx):
        self.dragging = val

    def on_slide(self, idx, value):
        if self.updating_sliders:
            return
        self.dragging = True
        self.target_deg = [float(sl.get()) for sl in self.sliders]
        for i, deg in enumerate(self.target_deg):
            self.value_labels[i]['text'] = f'{deg:.1f}°'
            self.angle_vars[i].set(f'{deg:.1f}')
        self.publish_target()
        self.update_kinematics_matrix()
        # Pequeño delay antes de permitir que refresh mueva los sliders
        self.root.after(100, lambda: setattr(self, 'dragging', False))

    def on_state(self, msg):
        # Solo guardamos el estado real para depuración; NO tocamos sliders.
        self.latest_state_deg = [math.degrees(p) for p in msg.position[:self.n]]

    def refresh(self):
        # Los sliders se quedan en la consigna del operador.
        self.updating_sliders = True
        for i in range(self.n):
            val = max(self.lower_deg[i],
                      min(self.upper_deg[i], self.target_deg[i]))
            self.sliders[i].set(val)
            self.value_labels[i]['text'] = f'{val:.1f}°'
        self.updating_sliders = False
        self.root.after(50, self.refresh)

    def apply_entry_angles(self):
        """Valida y aplica simultaneamente los angulos escritos."""
        try:
            values = [float(angle_var.get()) for angle_var in self.angle_vars]
        except ValueError:
            self.show_input_error('Escribe un numero valido en cada campo.')
            return

        for i, value in enumerate(values):
            if not math.isfinite(value):
                self.show_input_error(f'Junta {i}: el valor debe ser finito.')
                return
            if not self.lower_deg[i] <= value <= self.upper_deg[i]:
                self.show_input_error(
                    f'Junta {i}: usa un valor entre {self.lower_deg[i]:g}° '
                    f'y {self.upper_deg[i]:g}°.')
                return

        self.target_deg = values
        self.updating_sliders = True
        for i, value in enumerate(values):
            self.sliders[i].set(value)
            self.value_labels[i]['text'] = f'{value:.1f}°'
            self.angle_vars[i].set(f'{value:.1f}')
        self.updating_sliders = False
        self.publish_target()
        self.update_kinematics_matrix()
        self.input_status.configure(text='Angulos aplicados.', fg='dark green')

    def show_input_error(self, message):
        """Muestra un error de validacion sin enviar comandos al robot."""
        self.input_status.configure(text=message, fg='firebrick')

    def _map_to_servo_deg(self, joint_deg):
        """Convierte el grado del slider (offset respecto al home) en el
        angulo fisico absoluto del servo: slider 0 == home_angle_deg[i].
        Debe reflejar el rango real del servo (0-270 grados fisicos, ver
        SERVO_ANG_MIN/MAX en firmware/esp32_servos/src/main.cpp)."""
        servo_deg = []
        for i in range(self.n):
            physical = self.home_angle_deg[i] + joint_deg[i]
            physical = max(0.0, min(270.0, physical))
            servo_deg.append(round(physical, 1))
        return servo_deg

    def publish_target(self):
        """Publica la consigna angular actual en radianes (robot) y grados (servos)."""
        msg = RobotCommand()
        msg.mode = 1
        msg.position = [math.radians(value) for value in self.target_deg]
        msg.velocity = [0.0] * self.n
        self.cmd_pub.publish(msg)

        # Publicar tambien a los servos de la ESP32 (grados fisicos 0-270,
        # offset respecto a home_angle_deg)
        servo_msg = Float32MultiArray()
        servo_msg.data = self._map_to_servo_deg(self.target_deg)
        self.servo_pub.publish(servo_msg)

    def update_kinematics_matrix(self):
        """Recalcula y muestra 0A4 usando la posicion de los sliders."""
        if self.n != 3:
            return
        matrix = end_effector_matrix(self.target_deg, self.dh_link_lengths)
        for row in range(4):
            for column in range(4):
                value = matrix[row][column]
                if abs(value) < 0.00005:
                    value = 0.0
                self.matrix_labels[row][column]['text'] = f'{value: .4f}'

    def spin_ros(self):
        rclpy.spin(self)


def main(args=None):
    rclpy.init(args=args)
    node = GuiNode()
    # rclpy en hilo aparte para no bloquear la GUI
    t = threading.Thread(target=node.spin_ros, daemon=True)
    t.start()
    try:
        node.root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
