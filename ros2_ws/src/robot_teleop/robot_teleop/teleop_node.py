# Interfaz de barras deslizantes para teleoperar la pata.
# Cada slider es una junta: al moverlo publica /robot/command.
# El rango de cada slider es el limite de ese motor (del URDF / parametros).
#
# Como ROS y tkinter NO pueden correr en el mismo hilo, rclpy.spin va en un
# hilo aparte (daemon) y la GUI corre en el principal con root.mainloop().
# Los sliders se refrescan desde el hilo de la GUI con root.after(), no desde
# el callback de ROS, para que no se trabe (ese era el retardo).
import threading
import rclpy
from rclpy.node import Node
from robot_interfaces.msg import RobotCommand, JointState
import tkinter as tk


class GuiNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.declare_parameter('num_joints', 3)
        self.declare_parameter('joint_limits_lower', [-1.0, -1.0, -2.0])
        self.declare_parameter('joint_limits_upper', [1.0, 1.0, 0.0])
        self.n = self.get_parameter('num_joints').value
        self.lower = self.get_parameter('joint_limits_lower').value
        self.upper = self.get_parameter('joint_limits_upper').value

        self.cmd_pub = self.create_publisher(RobotCommand, '/robot/command', 10)
        self.create_subscription(JointState, '/robot/joint_states', self.on_state, 10)

        self.dragging = False          # True mientras el usuario mueve un slider
        self.latest_state = [0.0] * self.n
        self.root = tk.Tk()
        self.root.title('Teleop pata bipedo (sliders)')
        self.sliders = []
        for i in range(self.n):
            tk.Label(self.root, text=f'junta {i}').grid(row=i, column=0)
            s = tk.Scale(self.root, from_=self.lower[i], to=self.upper[i],
                         resolution=0.01, orient=tk.HORIZONTAL, length=300,
                         command=lambda v, idx=i: self.on_slide(idx, v))
            s.bind('<ButtonPress-1>', lambda e, idx=i: self.set_drag(True, idx))
            s.bind('<ButtonRelease-1>', lambda e, idx=i: self.set_drag(False, idx))
            s.grid(row=i, column=1)
            self.sliders.append(s)

        # Refresca los sliders desde el hilo de la GUI (no del callback ROS)
        self.root.after(50, self.refresh)

    def set_drag(self, val, idx):
        self.dragging = val

    def on_slide(self, idx, value):
        # El usuario movio el slider -> publicamos de inmediato
        msg = RobotCommand()
        msg.mode = 1
        msg.position = [float(sl.get()) for sl in self.sliders]
        msg.velocity = [0.0] * self.n
        self.cmd_pub.publish(msg)

    def on_state(self, msg):
        # Guardamos el estado; el refresh lo aplica en el hilo de la GUI
        self.latest_state = list(msg.position[:self.n])

    def refresh(self):
        if not self.dragging:
            for i in range(self.n):
                self.sliders[i].set(self.latest_state[i])
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
