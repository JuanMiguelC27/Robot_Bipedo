import math
import threading

import rclpy
from rclpy.node import Node

from robot_interfaces.msg import RobotCommand, JointState

import tkinter as tk
from tkinter import ttk

from robot_kinematics.kinem_leg_gen import (
    forward_kinematics_right,
    forward_kinematics_left,
    get_position,
    format_matrix
)


class GuiNode(Node):

    def __init__(self):
        super().__init__('teleop_node')

        # =====================================================
        # CONFIGURACIÓN
        # =====================================================

        self.declare_parameter(
            'num_joints',
            6
        )

        # Límites de los sliders en grados
        #
        # Orden:
        # 0 -> Right Hip Roll
        # 1 -> Right Hip Pitch
        # 2 -> Right Knee
        # 3 -> Left Hip Roll
        # 4 -> Left Hip Pitch
        # 5 -> Left Knee

        self.declare_parameter(
            'joint_limits_lower_deg',
            [
                -10.0, -105.0, -105.0,
                -10.0, -105.0, -105.0
            ]
        )

        self.declare_parameter(
            'joint_limits_upper_deg',
            [
                120.0, 105.0, 105.0,
                120.0, 105.0, 105.0
            ]
        )

        self.n = self.get_parameter(
            'num_joints'
        ).value

        self.lower_deg = list(
            self.get_parameter(
                'joint_limits_lower_deg'
            ).value
        )

        self.upper_deg = list(
            self.get_parameter(
                'joint_limits_upper_deg'
            ).value
        )

        # =====================================================
        # NOMBRES DE LAS ARTICULACIONES
        # =====================================================

        self.joint_names = [
            'Right_Hip_Roll_Joint',
            'Right_Hip_Pitch_Joint',
            'Right_Knee_Joint',
            'Left_Hip_Roll_Joint',
            'Left_Hip_Pitch_Joint',
            'Left_Knee_Joint'
        ]

        # =====================================================
        # ROS
        # =====================================================

        self.cmd_pub = self.create_publisher(
            RobotCommand,
            '/robot/command',
            10
        )

        self.create_subscription(
            JointState,
            '/robot/joint_states',
            self.on_state,
            10
        )

        # =====================================================
        # VARIABLES
        # =====================================================

        self.dragging = False
        self.updating_sliders = False

        self.target_deg = [0.0] * self.n
        self.latest_state_deg = [0.0] * self.n

        # Diccionarios para almacenar las MTH
        self.right_transformations = {}
        self.left_transformations = {}

        # =====================================================
        # VENTANA
        # =====================================================

        self.root = tk.Tk()
        self.root.title(
            'Teleop robot bípedo'
        )

        # =====================================================
        # TÍTULO
        # =====================================================

        tk.Label(
            self.root,
            text='TELEOPERACIÓN ROBOT BÍPEDO',
            font=('TkDefaultFont', 12, 'bold')
        ).grid(
            row=0,
            column=0,
            columnspan=3,
            pady=(10, 5)
        )

        # =====================================================
        # SLIDERS
        # =====================================================

        self.sliders = []
        self.value_labels = []

        self.create_leg_interface(
            title='PIERNA DERECHA',
            start_index=0,
            start_row=1
        )

        self.create_leg_interface(
            title='PIERNA IZQUIERDA',
            start_index=3,
            start_row=6
        )

        # =====================================================
        # ENTRADA MANUAL
        # =====================================================

        input_frame = tk.LabelFrame(
            self.root,
            text='Ingresar ángulos manualmente (grados)',
            padx=10,
            pady=8
        )

        input_frame.grid(
            row=10,
            column=0,
            columnspan=3,
            padx=10,
            pady=(10, 0),
            sticky='ew'
        )

        self.angle_vars = []
        self.angle_entries = []

        # ---------- DERECHA ----------

        tk.Label(
            input_frame,
            text='Derecha'
        ).grid(
            row=0,
            column=0,
            columnspan=6,
            pady=(2, 4)
        )

        for i in range(3):

            tk.Label(
                input_frame,
                text=self.short_name(i)
            ).grid(
                row=1,
                column=i * 2,
                padx=(4, 2)
            )

            angle_var = tk.StringVar(
                value='0.0'
            )

            entry = tk.Entry(
                input_frame,
                textvariable=angle_var,
                width=8,
                justify='right'
            )

            entry.grid(
                row=1,
                column=i * 2 + 1,
                padx=(0, 8)
            )

            self.angle_vars.append(
                angle_var
            )

            self.angle_entries.append(
                entry
            )

        # ---------- IZQUIERDA ----------

        tk.Label(
            input_frame,
            text='Izquierda'
        ).grid(
            row=2,
            column=0,
            columnspan=6,
            pady=(8, 4)
        )

        for i in range(3):

            idx = i + 3

            tk.Label(
                input_frame,
                text=self.short_name(idx)
            ).grid(
                row=3,
                column=i * 2,
                padx=(4, 2)
            )

            angle_var = tk.StringVar(
                value='0.0'
            )

            entry = tk.Entry(
                input_frame,
                textvariable=angle_var,
                width=8,
                justify='right'
            )

            entry.grid(
                row=3,
                column=i * 2 + 1,
                padx=(0, 8)
            )

            self.angle_vars.append(
                angle_var
            )

            self.angle_entries.append(
                entry
            )

        # ---------- BOTÓN ----------

        tk.Button(
            input_frame,
            text='Aplicar ángulos',
            command=self.apply_entry_angles
        ).grid(
            row=4,
            column=0,
            columnspan=6,
            pady=(10, 2)
        )

        self.input_status = tk.Label(
            input_frame,
            text='',
            anchor='w'
        )

        self.input_status.grid(
            row=5,
            column=0,
            columnspan=6,
            sticky='w',
            padx=4,
            pady=(4, 0)
        )

        # =====================================================
        # CINEMÁTICA DIRECTA
        # =====================================================

        self.create_kinematics_interface(
            self.root
        )

        # =====================================================
        # ACTUALIZACIÓN INICIAL DE MTH
        # =====================================================

        self.update_kinematics_display()

        # =====================================================
        # ACTUALIZACIÓN
        # =====================================================

        self.root.after(
            50,
            self.refresh
        )

    # =========================================================
    # CREAR INTERFAZ DE UNA PIERNA
    # =========================================================

    def create_leg_interface(
        self,
        title,
        start_index,
        start_row
    ):

        tk.Label(
            self.root,
            text=title,
            font=('TkDefaultFont', 10, 'bold')
        ).grid(
            row=start_row,
            column=0,
            columnspan=3,
            pady=(8, 2)
        )

        for local_index in range(3):

            i = start_index + local_index

            tk.Label(
                self.root,
                text=self.short_name(i)
            ).grid(
                row=start_row + 1 + local_index,
                column=0,
                padx=(10, 5)
            )

            s = tk.Scale(
                self.root,
                from_=self.lower_deg[i],
                to=self.upper_deg[i],
                resolution=0.5,
                orient=tk.HORIZONTAL,
                length=320,
                command=lambda value, idx=i:
                    self.on_slide(idx, value)
            )

            s.bind(
                '<ButtonPress-1>',
                lambda event, idx=i:
                    self.set_drag(True, idx)
            )

            s.bind(
                '<ButtonRelease-1>',
                lambda event, idx=i:
                    self.set_drag(False, idx)
            )

            s.grid(
                row=start_row + 1 + local_index,
                column=1
            )

            self.sliders.append(s)

            lbl = tk.Label(
                self.root,
                text='0.0°',
                width=8
            )

            lbl.grid(
                row=start_row + 1 + local_index,
                column=2
            )

            self.value_labels.append(
                lbl
            )

    # =========================================================
    # NOMBRES CORTOS
    # =========================================================

    def short_name(self, index):

        names = [
            'Hip Roll',
            'Hip Pitch',
            'Knee',
            'Hip Roll',
            'Hip Pitch',
            'Knee'
        ]

        return names[index]

    # =========================================================
    # INTERFAZ DE CINEMÁTICA
    # =========================================================

    def create_kinematics_interface(
        self,
        parent
    ):

        frame = tk.LabelFrame(
            parent,
            text='CINEMÁTICA DIRECTA - MATRICES MTH',
            padx=8,
            pady=8
        )

        frame.grid(
            row=11,
            column=0,
            columnspan=3,
            padx=10,
            pady=(8, 10),
            sticky='ew'
        )

        # =====================================================
        # CONFIGURAR COLUMNAS
        # =====================================================

        frame.grid_columnconfigure(
            0,
            weight=1
        )

        frame.grid_columnconfigure(
            1,
            weight=1
        )

        # =====================================================
        # PIERNA DERECHA
        # =====================================================

        right_frame = tk.Frame(
            frame
        )

        right_frame.grid(
            row=0,
            column=0,
            padx=10,
            sticky='n'
        )

        tk.Label(
            right_frame,
            text='PIERNA DERECHA',
            font=('TkDefaultFont', 10, 'bold')
        ).pack(
            pady=(0, 4)
        )

        tk.Label(
            right_frame,
            text='Seleccionar transformación:'
        ).pack(
            pady=(0, 2)
        )

        self.right_transform_var = tk.StringVar(
            value='T04'
        )

        self.right_transform_selector = ttk.Combobox(
            right_frame,
            textvariable=self.right_transform_var,
            values=[
                'T01',
                'T02',
                'T03',
                'T04'
            ],
            state='readonly',
            width=12
        )

        self.right_transform_selector.pack(
            pady=(0, 5)
        )

        self.right_transform_selector.bind(
            '<<ComboboxSelected>>',
            self.update_kinematics_display
        )

        self.right_matrix_text = tk.Text(
            right_frame,
            width=38,
            height=5,
            font=('Courier New', 9),
            wrap='none'
        )

        self.right_matrix_text.pack(
            pady=4
        )

        self.right_matrix_text.configure(
            state='disabled'
        )

        self.right_position_label = tk.Label(
            right_frame,
            text=(
                'Posición: '
                'X = 0.0000   '
                'Y = 0.0000   '
                'Z = 0.0000'
            ),
            font=('Courier New', 8)
        )

        self.right_position_label.pack(
            pady=(2, 0)
        )

        # =====================================================
        # PIERNA IZQUIERDA
        # =====================================================

        left_frame = tk.Frame(
            frame
        )

        left_frame.grid(
            row=0,
            column=1,
            padx=10,
            sticky='n'
        )

        tk.Label(
            left_frame,
            text='PIERNA IZQUIERDA',
            font=('TkDefaultFont', 10, 'bold')
        ).pack(
            pady=(0, 4)
        )

        tk.Label(
            left_frame,
            text='Seleccionar transformación:'
        ).pack(
            pady=(0, 2)
        )

        self.left_transform_var = tk.StringVar(
            value='T04'
        )

        self.left_transform_selector = ttk.Combobox(
            left_frame,
            textvariable=self.left_transform_var,
            values=[
                'T01',
                'T02',
                'T03',
                'T04'
            ],
            state='readonly',
            width=12
        )

        self.left_transform_selector.pack(
            pady=(0, 5)
        )

        self.left_transform_selector.bind(
            '<<ComboboxSelected>>',
            self.update_kinematics_display
        )

        self.left_matrix_text = tk.Text(
            left_frame,
            width=38,
            height=5,
            font=('Courier New', 9),
            wrap='none'
        )

        self.left_matrix_text.pack(
            pady=4
        )

        self.left_matrix_text.configure(
            state='disabled'
        )

        self.left_position_label = tk.Label(
            left_frame,
            text=(
                'Posición: '
                'X = 0.0000   '
                'Y = 0.0000   '
                'Z = 0.0000'
            ),
            font=('Courier New', 8)
        )

        self.left_position_label.pack(
            pady=(2, 0)
        )

    # =========================================================
    # ACTUALIZAR CINEMÁTICA
    # =========================================================

    def update_kinematics_display(
        self,
        event=None
    ):

        try:

            # -------------------------------------------------
            # OBTENER ÁNGULOS ACTUALES
            # -------------------------------------------------

            q_right = self.target_deg[0:3]
            q_left = self.target_deg[3:6]

            # -------------------------------------------------
            # CALCULAR TODAS LAS MTH
            # -------------------------------------------------

            right = forward_kinematics_right(
                q_right
            )

            left = forward_kinematics_left(
                q_left
            )

            # -------------------------------------------------
            # GUARDAR TRANSFORMACIONES DERECHA
            # -------------------------------------------------

            self.right_transformations = {
                'T01': right[0],
                'T02': right[1],
                'T03': right[2],
                'T04': right[3]
            }

            # -------------------------------------------------
            # GUARDAR TRANSFORMACIONES IZQUIERDA
            # -------------------------------------------------

            self.left_transformations = {
                'T01': left[0],
                'T02': left[1],
                'T03': left[2],
                'T04': left[3]
            }

            # -------------------------------------------------
            # OBTENER SELECCIÓN
            # -------------------------------------------------

            right_name = (
                self.right_transform_var.get()
            )

            left_name = (
                self.left_transform_var.get()
            )

            T_right = (
                self.right_transformations[
                    right_name
                ]
            )

            T_left = (
                self.left_transformations[
                    left_name
                ]
            )

            # -------------------------------------------------
            # MOSTRAR MTH DERECHA
            # -------------------------------------------------

            self.right_matrix_text.configure(
                state='normal'
            )

            self.right_matrix_text.delete(
                '1.0',
                tk.END
            )

            self.right_matrix_text.insert(
                tk.END,
                format_matrix(T_right)
            )

            self.right_matrix_text.configure(
                state='disabled'
            )

            # -------------------------------------------------
            # MOSTRAR MTH IZQUIERDA
            # -------------------------------------------------

            self.left_matrix_text.configure(
                state='normal'
            )

            self.left_matrix_text.delete(
                '1.0',
                tk.END
            )

            self.left_matrix_text.insert(
                tk.END,
                format_matrix(T_left)
            )

            self.left_matrix_text.configure(
                state='disabled'
            )

            # -------------------------------------------------
            # OBTENER POSICIONES
            # -------------------------------------------------

            pos_right = get_position(
                T_right
            )

            pos_left = get_position(
                T_left
            )

            # -------------------------------------------------
            # MOSTRAR POSICIÓN DERECHA
            # -------------------------------------------------

            self.right_position_label.configure(
                text=(
                    f'Posición: '
                    f'X = {pos_right[0]:.4f}   '
                    f'Y = {pos_right[1]:.4f}   '
                    f'Z = {pos_right[2]:.4f}'
                )
            )

            # -------------------------------------------------
            # MOSTRAR POSICIÓN IZQUIERDA
            # -------------------------------------------------

            self.left_position_label.configure(
                text=(
                    f'Posición: '
                    f'X = {pos_left[0]:.4f}   '
                    f'Y = {pos_left[1]:.4f}   '
                    f'Z = {pos_left[2]:.4f}'
                )
            )

        except Exception as e:

            self.get_logger().error(
                f'Error calculando cinemática: {e}'
            )

    # =========================================================
    # SLIDER
    # =========================================================

    def set_drag(
        self,
        val,
        idx
    ):

        self.dragging = val

    def on_slide(
        self,
        idx,
        value
    ):

        if self.updating_sliders:
            return

        self.dragging = True

        self.target_deg = [
            float(sl.get())
            for sl in self.sliders
        ]

        for i, deg in enumerate(
            self.target_deg
        ):

            self.value_labels[i][
                'text'
            ] = f'{deg:.1f}°'

            self.angle_vars[i].set(
                f'{deg:.1f}'
            )

        # Publicar comando ROS
        self.publish_target()

        # Actualizar MTH
        self.update_kinematics_display()

        self.root.after(
            100,
            lambda: setattr(
                self,
                'dragging',
                False
            )
        )

    # =========================================================
    # ESTADO ROS
    # =========================================================

    def on_state(
        self,
        msg
    ):

        self.latest_state_deg = [
            math.degrees(p)
            for p in msg.position[:self.n]
        ]

    # =========================================================
    # REFRESCAR SLIDERS
    # =========================================================

    def refresh(self):

        self.updating_sliders = True

        for i in range(
            self.n
        ):

            val = max(
                self.lower_deg[i],
                min(
                    self.upper_deg[i],
                    self.target_deg[i]
                )
            )

            self.sliders[i].set(
                val
            )

            self.value_labels[i][
                'text'
            ] = f'{val:.1f}°'

        self.updating_sliders = False

        self.root.after(
            50,
            self.refresh
        )

    # =========================================================
    # ENTRADA MANUAL
    # =========================================================

    def apply_entry_angles(self):

        try:

            values = [
                float(angle_var.get())
                for angle_var in self.angle_vars
            ]

        except ValueError:

            self.show_input_error(
                'Escribe un número válido en cada campo.'
            )

            return

        # -----------------------------------------------------
        # VALIDAR LÍMITES
        # -----------------------------------------------------

        for i, value in enumerate(
            values
        ):

            if not math.isfinite(
                value
            ):

                self.show_input_error(
                    f'Junta {i}: el valor debe ser finito.'
                )

                return

            if not (
                self.lower_deg[i]
                <= value
                <= self.upper_deg[i]
            ):

                self.show_input_error(
                    f'Junta {i}: usa un valor entre '
                    f'{self.lower_deg[i]:g}° y '
                    f'{self.upper_deg[i]:g}°.'
                )

                return

        # -----------------------------------------------------
        # ACTUALIZAR VALORES
        # -----------------------------------------------------

        self.target_deg = values

        self.updating_sliders = True

        for i, value in enumerate(
            values
        ):

            self.sliders[i].set(
                value
            )

            self.value_labels[i][
                'text'
            ] = f'{value:.1f}°'

            self.angle_vars[i].set(
                f'{value:.1f}'
            )

        self.updating_sliders = False

        # -----------------------------------------------------
        # PUBLICAR
        # -----------------------------------------------------

        self.publish_target()

        # -----------------------------------------------------
        # ACTUALIZAR CINEMÁTICA
        # -----------------------------------------------------

        self.update_kinematics_display()

        self.input_status.configure(
            text='Ángulos aplicados.',
            fg='dark green'
        )

    # =========================================================
    # MENSAJE DE ERROR
    # =========================================================

    def show_input_error(
        self,
        message
    ):

        self.input_status.configure(
            text=message,
            fg='firebrick'
        )

    # =========================================================
    # PUBLICAR
    # =========================================================

    def publish_target(self):

        msg = RobotCommand()

        msg.mode = 1

        msg.position = [
            math.radians(value)
            for value in self.target_deg
        ]

        msg.velocity = [
            0.0
        ] * self.n

        self.cmd_pub.publish(
            msg
        )

    # =========================================================
    # ROS
    # =========================================================

    def spin_ros(self):

        rclpy.spin(
            self
        )


# =============================================================
# MAIN
# =============================================================

def main(
    args=None
):

    rclpy.init(
        args=args
    )

    node = GuiNode()

    t = threading.Thread(
        target=node.spin_ros,
        daemon=True
    )

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
