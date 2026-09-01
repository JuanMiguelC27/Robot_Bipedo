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
import tkinter as tk


class GuiNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.declare_parameter('num_joints', 3)
        # Servos de 270° -> rango util -135° a +135° por slider.
        self.declare_parameter('joint_limits_lower_deg', [-135.0, -135.0, -135.0])
        self.declare_parameter('joint_limits_upper_deg', [ 135.0,  135.0,  135.0])
        self.n = self.get_parameter('num_joints').value
        self.lower_deg = self.get_parameter('joint_limits_lower_deg').value
        self.upper_deg = self.get_parameter('joint_limits_upper_deg').value

        self.cmd_pub = self.create_publisher(RobotCommand, '/robot/command', 10)
        self.create_subscription(JointState, '/robot/joint_states', self.on_state, 10)

        self.dragging = False
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

        self.root.after(50, self.refresh)

    def set_drag(self, val, idx):
        self.dragging = val

    def on_slide(self, idx, value):
        self.dragging = True
        self.target_deg = [float(sl.get()) for sl in self.sliders]
        rads = [math.radians(d) for d in self.target_deg]
        msg = RobotCommand()
        msg.mode = 1
        msg.position = rads
        msg.velocity = [0.0] * self.n
        self.cmd_pub.publish(msg)
        for i, deg in enumerate(self.target_deg):
            self.value_labels[i]['text'] = f'{deg:.1f}°'
        # Pequeño delay antes de permitir que refresh mueva los sliders
        self.root.after(100, lambda: setattr(self, 'dragging', False))

    def on_state(self, msg):
        # Solo guardamos el estado real para depuración; NO tocamos sliders.
        self.latest_state_deg = [math.degrees(p) for p in msg.position[:self.n]]

    def refresh(self):
        # Los sliders se quedan en la consigna del operador.
        for i in range(self.n):
            val = max(self.lower_deg[i],
                      min(self.upper_deg[i], self.target_deg[i]))
            self.sliders[i].set(val)
            self.value_labels[i]['text'] = f'{val:.1f}°'
        self.root.after(50, self.refresh)

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
