# ============================================================
# TELEOP NODE - ROBOT BÍPEDO
# ============================================================
#
# Este nodo proporciona una interfaz gráfica en Tkinter para:
#
#   1. Controlar las 3 articulaciones de una pierna.
#   2. Introducir manualmente los ángulos articulares.
#   3. Publicar los comandos mediante ROS 2.
#   4. Recibir el estado actual de las articulaciones.
#   5. Calcular la cinemática directa.
#   6. Mostrar las matrices homogéneas T01, T02, T03 y T04.
#   7. Mostrar la posición X, Y, Z del extremo de la cadena.
#
# La misma GUI se utiliza para ambas piernas.
#
# El parámetro:
#
#       leg_side = "right"
#       leg_side = "left"
#
# determina qué cadena cinemática utilizar y qué nombre
# mostrar en la interfaz.
#
# ============================================================


# ------------------------------------------------------------
# IMPORTACIONES
# ------------------------------------------------------------

import math
import threading

import rclpy
from rclpy.node import Node

from robot_interfaces.msg import RobotCommand, JointState
from std_msgs.msg import Float32MultiArray

import tkinter as tk
from tkinter import ttk


# ------------------------------------------------------------
# IMPORTACIÓN DE CINEMÁTICA
# ------------------------------------------------------------

from robot_kinematics.kinem_leg_gen import (
    forward_kinematics_right,
    forward_kinematics_left,
    get_position
)


# ============================================================
# CLASE PRINCIPAL
# ============================================================

class TeleopNode(Node):

    def __init__(self):

        # ----------------------------------------------------
        # Inicialización del nodo ROS 2
        # ----------------------------------------------------

        super().__init__('teleop_node')


        # ----------------------------------------------------
        # PARÁMETROS ROS 2
        # ----------------------------------------------------

        self.declare_parameter(
            'leg_side',
            'right'
        )

        self.declare_parameter(
            'num_joints',
            3
        )

        self.leg_side = (
            self.get_parameter('leg_side')
            .get_parameter_value()
            .string_value
            .lower()
        )

        self.num_joints = (
            self.get_parameter('num_joints')
            .get_parameter_value()
            .integer_value
        )


        # ----------------------------------------------------
        # VALIDACIÓN DE LA PIERNA
        # ----------------------------------------------------

        if self.leg_side not in ['right', 'left']:

            self.get_logger().warn(
                f"leg_side='{self.leg_side}' no válido. "
                f"Se utilizará 'right'."
            )

            self.leg_side = 'right'


        # Nombre utilizado visualmente en la interfaz.

        if self.leg_side == 'right':
            self.leg_name = 'DERECHA'
        else:
            self.leg_name = 'IZQUIERDA'


        # ====================================================
        # CONFIGURACIÓN DE ARTICULACIONES
        # ====================================================

        self.joint_names = [
            'Hip Roll',
            'Hip Pitch',
            'Knee Pitch'
        ]


        # ----------------------------------------------------
        # LÍMITES VISUALES / DE ENTRADA
        # ----------------------------------------------------

        self.lower_deg = [
            -15.0,    # Hip Roll
            -90.0,    # Hip Pitch
            -90.0     # Knee Pitch
        ]

        self.upper_deg = [
            90.0,     # Hip Roll
            90.0,     # Hip Pitch
            90.0      # Knee Pitch
        ]


        # ----------------------------------------------------
        # OFFSETS DE LOS SERVOS
        # ----------------------------------------------------

        self.servo_offset_deg = [
            180.0,    # Hip Roll
            135.0,    # Hip Pitch
            135.0     # Knee
        ]


        # ====================================================
        # VALORES ACTUALES Y OBJETIVO
        # ====================================================

        self.target_deg = [
            0.0,
            0.0,
            0.0
        ]

        self.current_deg = [
            0.0,
            0.0,
            0.0
        ]


        # ----------------------------------------------------
        # ÁNGULO REAL DEL SERVO (feedback desde el ESP32,
        # tópico /servo_states). Se guarda ya convertido al
        # mismo sistema de grados que usan los sliders (se le
        # resta el offset de servo) para poder comparar
        # "comandado" vs "real" de forma directa.
        # ----------------------------------------------------

        self.real_deg = [
            0.0,
            0.0,
            0.0
        ]

        self.has_real_state = False


        # ====================================================
        # VARIABLES DE CINEMÁTICA
        # ====================================================

        self.transforms = []

        # MTH seleccionada actualmente.

        self.selected_mth = 'T04'


        # ====================================================
        # PUBLICADORES ROS 2
        # ====================================================

        # Publicador principal de comandos del robot.

        self.command_pub = self.create_publisher(
            RobotCommand,
            '/robot/command',
            10
        )


        # Publicador utilizado para enviar los ángulos
        # a los servos en grados.

        self.servo_pub = self.create_publisher(
            Float32MultiArray,
            '/servo_commands',
            10
        )


        # ====================================================
        # SUSCRIPTOR DE ESTADO
        # ====================================================

        self.state_sub = self.create_subscription(
            JointState,
            '/robot/joint_states',
            self.on_state,
            10
        )


        # Estado real de los servos, publicado por el firmware
        # del ESP32 (ver firmware/esp32_servos/src/main.cpp).

        self.servo_state_sub = self.create_subscription(
            Float32MultiArray,
            '/servo_states',
            self.on_servo_state,
            10
        )


        # ====================================================
        # CREACIÓN DE LA INTERFAZ
        # ====================================================

        self.root = tk.Tk()

        self.root.title(
            f"Teleop pata bípedo - Pierna {self.leg_name}"
        )


        # ----------------------------------------------------
        # Tamaño inicial de la ventana
        # ----------------------------------------------------

        self.root.geometry("780x740")

        self.root.resizable(
            False,
            True
        )


        # ====================================================
        # ESTILO DE TKINTER
        # ====================================================

        style = ttk.Style()

        try:
            style.theme_use('clam')
        except tk.TclError:
            pass


        style.configure(
            'Title.TLabel',
            font=('Arial', 18, 'bold')
        )

        style.configure(
            'Subtitle.TLabel',
            font=('Arial', 11)
        )

        style.configure(
            'Section.TLabelframe.Label',
            font=('Arial', 11, 'bold')
        )


        # ====================================================
        # CONTENEDOR PRINCIPAL
        # ====================================================

        main_frame = ttk.Frame(
            self.root,
            padding=8
        )

        main_frame.pack(
            fill='both',
            expand=True
        )


        # ====================================================
        # TÍTULO
        # ====================================================

        title_label = ttk.Label(
            main_frame,
            text=f"TELEOPERACIÓN PIERNA {self.leg_name}",
            style='Title.TLabel'
        )

        title_label.pack(
            pady=(0, 2)
        )


        subtitle_label = ttk.Label(
            main_frame,
            text="Control de articulaciones y cinemática directa",
            style='Subtitle.TLabel'
        )

        subtitle_label.pack(
            pady=(0, 8)
        )


        # ====================================================
        # SECCIÓN DE ARTICULACIONES
        # ====================================================

        joints_frame = ttk.LabelFrame(
            main_frame,
            text=f"Articulaciones - Pierna {self.leg_name}",
            padding=8,
            style='Section.TLabelframe'
        )

        joints_frame.pack(
            fill='x',
            pady=(0, 8)
        )


        # Listas donde almacenaremos los elementos gráficos.

        self.sliders = []
        self.slider_labels = []
        self.real_labels = []


        # ----------------------------------------------------
        # CREACIÓN DE LOS 3 SLIDERS
        # ----------------------------------------------------

        for i in range(self.num_joints):

            name_label = ttk.Label(
                joints_frame,
                text=self.joint_names[i],
                width=12
            )

            name_label.grid(
                row=i,
                column=0,
                padx=(8, 5),
                pady=5,
                sticky='w'
            )


            # ------------------------------------------------
            # SLIDER
            # ------------------------------------------------

            slider = tk.Scale(
                joints_frame,
                from_=self.lower_deg[i],
                to=self.upper_deg[i],
                orient='horizontal',
                resolution=0.1,
                showvalue=False,
                length=350,
                command=lambda value, index=i:
                    self.on_slider_change(
                        index,
                        value
                    )
            )

            slider.set(
                self.target_deg[i]
            )

            slider.grid(
                row=i,
                column=1,
                padx=5,
                pady=2
            )


            self.sliders.append(
                slider
            )


            # ------------------------------------------------
            # ETIQUETA DEL VALOR DEL SLIDER
            # ------------------------------------------------

            value_label = ttk.Label(
                joints_frame,
                text=f"{self.target_deg[i]:.1f}°",
                width=7
            )

            value_label.grid(
                row=i,
                column=2,
                padx=(5, 8),
                pady=5
            )


            self.slider_labels.append(
                value_label
            )


            # ------------------------------------------------
            # ETIQUETA DEL ÁNGULO REAL (feedback del servo)
            # ------------------------------------------------

            real_label = ttk.Label(
                joints_frame,
                text="real: sin datos",
                width=16,
                foreground='gray'
            )

            real_label.grid(
                row=i,
                column=3,
                padx=(5, 8),
                pady=5
            )

            self.real_labels.append(
                real_label
            )


        # ====================================================
        # INGRESO MANUAL
        # ====================================================

        manual_frame = ttk.LabelFrame(
            main_frame,
            text="Ingresar ángulos manualmente (grados)",
            padding=8,
            style='Section.TLabelframe'
        )

        manual_frame.pack(
            fill='x',
            pady=(0, 8)
        )


        # ----------------------------------------------------
        # CAMPOS DE ENTRADA
        # ----------------------------------------------------

        self.entries = []

        for i in range(self.num_joints):

            label = ttk.Label(
                manual_frame,
                text=self.joint_names[i]
            )

            label.grid(
                row=0,
                column=i * 2,
                padx=(5, 3),
                pady=3
            )


            entry = tk.Entry(
                manual_frame,
                width=8,
                justify='center'
            )

            entry.insert(
                0,
                f"{self.target_deg[i]:.1f}"
            )

            entry.grid(
                row=0,
                column=i * 2 + 1,
                padx=(0, 8),
                pady=3
            )


            self.entries.append(
                entry
            )


        # ====================================================
        # BOTÓN APLICAR
        # ====================================================

        apply_button = ttk.Button(
            manual_frame,
            text="Aplicar ángulos",
            command=self.apply_entry_angles
        )

        apply_button.grid(
            row=1,
            column=0,
            columnspan=6,
            pady=(8, 4)
        )


        # ====================================================
        # BOTÓN APLICAR A HARDWARE (ESP32 real)
        # ====================================================

        hardware_button = tk.Button(
            manual_frame,
            text="Aplicar a motores (HARDWARE)",
            bg='#ffdddd',
            command=self.apply_to_hardware
        )

        hardware_button.grid(
            row=2,
            column=0,
            columnspan=6,
            pady=(4, 4)
        )


        # ====================================================
        # MENSAJE DE ESTADO
        # ====================================================

        self.status_label = tk.Label(
            manual_frame,
            text="",
            font=('Arial', 10, 'bold'),
            anchor='center'
        )

        self.status_label.grid(
            row=3,
            column=0,
            columnspan=6,
            pady=(2, 0)
        )


        # ====================================================
        # CINEMÁTICA DIRECTA
        # ====================================================

        kinematics_frame = ttk.LabelFrame(
            main_frame,
            text="Cinemática directa",
            padding=8,
            style='Section.TLabelframe'
        )

        kinematics_frame.pack(
            fill='x',
            pady=(0, 0)
        )


        # ====================================================
        # SELECTOR DE MTH
        # ====================================================

        mth_label = ttk.Label(
            kinematics_frame,
            text="Seleccionar MTH:"
        )

        mth_label.grid(
            row=0,
            column=0,
            padx=(5, 5),
            pady=3,
            sticky='w'
        )


        self.mth_selector = ttk.Combobox(
            kinematics_frame,
            values=[
                'T01',
                'T02',
                'T03',
                'T04'
            ],
            state='readonly',
            width=10
        )

        self.mth_selector.set(
            self.selected_mth
        )

        self.mth_selector.grid(
            row=0,
            column=1,
            padx=5,
            pady=3,
            sticky='w'
        )


        # Cada vez que cambia la MTH seleccionada,
        # actualizamos la matriz y la posición.

        self.mth_selector.bind(
            '<<ComboboxSelected>>',
            self.on_mth_selected
        )


        # ====================================================
        # MATRIZ HOMOGÉNEA
        # ====================================================

        matrix_frame = ttk.Frame(
            kinematics_frame
        )

        matrix_frame.grid(
            row=1,
            column=0,
            columnspan=2,
            pady=(8, 5)
        )


        self.matrix_labels = []


        # ----------------------------------------------------
        # Creamos una matriz visual de 4x4.
        # ----------------------------------------------------

        for row in range(4):

            matrix_row = []

            for col in range(4):

                label = tk.Label(
                    matrix_frame,
                    text="0.0000",
                    width=12,
                    relief='ridge',
                    borderwidth=1,
                    anchor='center'
                )

                label.grid(
                    row=row,
                    column=col,
                    padx=1,
                    pady=1
                )

                matrix_row.append(
                    label
                )

            self.matrix_labels.append(
                matrix_row
            )


        # ====================================================
        # POSICIÓN
        # ====================================================

        position_frame = ttk.LabelFrame(
            kinematics_frame,
            text="Posición",
            padding=8,
            style='Section.TLabelframe'
        )

        position_frame.grid(
            row=2,
            column=0,
            columnspan=2,
            sticky='ew',
            pady=(8, 0)
        )


        # ----------------------------------------------------
        # Etiquetas X, Y y Z
        # ----------------------------------------------------

        self.position_labels = {}


        self.position_labels['x'] = ttk.Label(
            position_frame,
            text="x = 0.0000 m"
        )

        self.position_labels['x'].grid(
            row=0,
            column=0,
            padx=(20, 30),
            pady=2
        )


        self.position_labels['y'] = ttk.Label(
            position_frame,
            text="y = 0.0000 m"
        )

        self.position_labels['y'].grid(
            row=0,
            column=1,
            padx=30,
            pady=2
        )


        self.position_labels['z'] = ttk.Label(
            position_frame,
            text="z = 0.0000 m"
        )

        self.position_labels['z'].grid(
            row=0,
            column=2,
            padx=(30, 20),
            pady=2
        )


        # ====================================================
        # ACTUALIZACIÓN INICIAL DE CINEMÁTICA
        # ====================================================

        self.update_kinematics()


        # ====================================================
        # ACTUALIZACIÓN PERIÓDICA DE LA GUI
        # ====================================================

        self.root.after(
            50,
            self.refresh
        )


        # ====================================================
        # CIERRE DE LA VENTANA
        # ====================================================

        self.root.protocol(
            "WM_DELETE_WINDOW",
            self.on_close
        )


    # ========================================================
    # SLIDER
    # ========================================================

    def on_slider_change(
        self,
        index,
        value
    ):

        try:
            value = float(value)

        except ValueError:
            return


        # Guardamos el nuevo valor.

        self.target_deg[index] = value


        # Actualizamos el texto del slider.

        self.slider_labels[index].config(
            text=f"{value:.1f}°"
        )


        # Actualizamos el campo de entrada manual.

        self.entries[index].delete(
            0,
            tk.END
        )

        self.entries[index].insert(
            0,
            f"{value:.1f}"
        )


        # Publicamos el comando.

        self.publish_target()


        # Recalculamos la cinemática.

        self.update_kinematics()


    # ========================================================
    # APLICAR ÁNGULOS MANUALES
    # ========================================================

    def apply_entry_angles(self):

        # ----------------------------------------------------
        # 1. Intentamos convertir todos los campos a float.
        # ----------------------------------------------------

        try:

            values = [
                float(
                    self.entries[i].get()
                )
                for i in range(
                    self.num_joints
                )
            ]

        except ValueError:

            self.set_status(
                "Error: todos los ángulos deben ser valores numéricos.",
                "red"
            )

            return


        # ----------------------------------------------------
        # 2. VALIDAMOS LOS LÍMITES
        # ----------------------------------------------------

        for i, value in enumerate(values):

            if (
                value < self.lower_deg[i]
                or
                value > self.upper_deg[i]
            ):

                self.set_status(
                    f"{self.joint_names[i]} debe estar entre "
                    f"{self.lower_deg[i]:.1f}° y "
                    f"{self.upper_deg[i]:.1f}°.",
                    "red"
                )

                return


        # ----------------------------------------------------
        # 3. TODOS LOS VALORES SON VÁLIDOS
        # ----------------------------------------------------

        self.target_deg = values.copy()


        # ----------------------------------------------------
        # Actualizamos sliders y etiquetas.
        # ----------------------------------------------------

        for i, value in enumerate(values):

            self.sliders[i].set(
                value
            )

            self.slider_labels[i].config(
                text=f"{value:.1f}°"
            )


        # ----------------------------------------------------
        # 4. Publicamos el nuevo comando.
        # ----------------------------------------------------

        self.publish_target()


        # ----------------------------------------------------
        # 5. Recalculamos la cinemática.
        # ----------------------------------------------------

        self.update_kinematics()


        # ----------------------------------------------------
        # 6. Mensaje de éxito.
        # ----------------------------------------------------

        self.set_status(
            "✓ Ángulos aplicados correctamente.",
            "green"
        )


    # ========================================================
    # MENSAJE DE ESTADO
    # ========================================================

    def set_status(
        self,
        message,
        color
    ):

        self.status_label.config(
            text=message,
            foreground=color
        )


    # ========================================================
    # PUBLICAR COMANDO
    # ========================================================

    def publish_target(self):

        # ----------------------------------------------------
        # Conversión grados -> radianes
        # ----------------------------------------------------

        angles_rad = [
            math.radians(angle)
            for angle in self.target_deg
        ]


        # ----------------------------------------------------
        # Mensaje RobotCommand (preview, siempre en vivo).
        #
        # OJO: esto ya NO toca los motores reales. Solo mueve
        # el modelo en RViz. Para mandar la orden al hardware
        # hay que presionar el botón "Aplicar a motores".
        # ----------------------------------------------------

        command_msg = RobotCommand()

        command_msg.position = angles_rad

        self.command_pub.publish(
            command_msg
        )


    # ========================================================
    # APLICAR A HARDWARE (ESP32 real, vía /servo_commands)
    # ========================================================

    def apply_to_hardware(self):

        servo_msg = Float32MultiArray()

        servo_msg.data = [
            float(
                self.target_deg[i]
                + self.servo_offset_deg[i]
            )
            for i in range(
                self.num_joints
            )
        ]

        self.servo_pub.publish(
            servo_msg
        )

        self.set_status(
            "✓ Enviado al motor real (última consigna de los sliders).",
            "green"
        )


    # ========================================================
    # CINEMÁTICA DIRECTA
    # ========================================================

    def update_kinematics(self):

        q = self.target_deg.copy()


        # ----------------------------------------------------
        # Selección de la cadena cinemática
        # ----------------------------------------------------

        if self.leg_side == 'right':

            self.transforms = (
                forward_kinematics_right(q)
            )

        else:

            self.transforms = (
                forward_kinematics_left(q)
            )


        # ----------------------------------------------------
        # Actualizamos la representación gráfica.
        # ----------------------------------------------------

        self.update_kinematics_display()


    # ========================================================
    # ACTUALIZACIÓN DE MATRIZ Y POSICIÓN
    # ========================================================

    def update_kinematics_display(self):

        # ----------------------------------------------------
        # Si todavía no hay matrices calculadas, no hacemos
        # nada.
        # ----------------------------------------------------

        if not self.transforms:
            return


        # ----------------------------------------------------
        # Determinamos qué índice corresponde a la MTH.
        #
        # T01 -> índice 0
        # T02 -> índice 1
        # T03 -> índice 2
        # T04 -> índice 3
        # ----------------------------------------------------

        mth_index = int(
            self.selected_mth[-1]
        ) - 1


        # Verificación para evitar acceder a una posición
        # inexistente.

        if (
            mth_index < 0
            or
            mth_index >= len(self.transforms)
        ):
            return


        T = self.transforms[mth_index]


        # ====================================================
        # ACTUALIZAR MATRIZ
        # ====================================================

        for row in range(4):

            for col in range(4):

                value = T[row, col]

                self.matrix_labels[row][col].config(
                    text=f"{value:.4f}"
                )


        # ====================================================
        # EXTRAER POSICIÓN
        # ====================================================

        position = get_position(T)

        x = position[0]
        y = position[1]
        z = position[2]


        self.position_labels['x'].config(
            text=f"x = {x:.4f} m"
        )

        self.position_labels['y'].config(
            text=f"y = {y:.4f} m"
        )

        self.position_labels['z'].config(
            text=f"z = {z:.4f} m"
        )


    # ========================================================
    # CAMBIO DE MTH
    # ========================================================

    def on_mth_selected(
        self,
        event=None
    ):

        self.selected_mth = (
            self.mth_selector.get()
        )

        self.update_kinematics_display()


    # ========================================================
    # RECIBIR ESTADO DE ARTICULACIONES
    # ========================================================

    def on_state(
        self,
        msg
    ):

        try:

            positions = msg.position

            for i in range(
                min(
                    self.num_joints,
                    len(positions)
                )
            ):

                self.current_deg[i] = (
                    math.degrees(
                        positions[i]
                    )
                )

        except Exception as e:

            self.get_logger().error(
                f"Error procesando JointState: {e}"
            )


    # ========================================================
    # RECIBIR ÁNGULO REAL DEL SERVO (/servo_states, ESP32)
    # ========================================================

    def on_servo_state(
        self,
        msg
    ):

        try:

            data = msg.data

            n = min(
                self.num_joints,
                len(data)
            )

            for i in range(n):

                # El firmware trabaja en "espacio de servo"
                # (con el offset ya sumado). Se resta acá para
                # mostrarlo en el mismo sistema de grados que
                # usan los sliders (espacio cinemático).

                self.real_deg[i] = (
                    float(data[i])
                    - self.servo_offset_deg[i]
                )

            self.has_real_state = True

        except Exception as e:

            self.get_logger().error(
                f"Error procesando /servo_states: {e}"
            )


    # ========================================================
    # ACTUALIZACIÓN PERIÓDICA DE LA GUI
    # ========================================================

    def refresh(self):

        # ----------------------------------------------------
        # Reflejamos en la GUI el último ángulo real recibido
        # del servo (o "sin datos" si todavía no llegó nada).
        #
        # Esto se hace acá, en el hilo de Tkinter, y no dentro
        # de on_servo_state (que corre en el hilo de ROS), para
        # no tocar widgets desde otro hilo.
        # ----------------------------------------------------

        for i in range(self.num_joints):

            if self.has_real_state:

                diff = abs(
                    self.real_deg[i]
                    - self.target_deg[i]
                )

                color = 'dark green' if diff < 2.0 else 'firebrick'

                self.real_labels[i].config(
                    text=f"real: {self.real_deg[i]:.1f}°",
                    foreground=color
                )

            else:

                self.real_labels[i].config(
                    text="real: sin datos",
                    foreground='gray'
                )

        self.root.after(
            50,
            self.refresh
        )


    # ========================================================
    # CIERRE
    # ========================================================

    def on_close(self):

        try:

            self.destroy_node()

        except Exception:
            pass


        try:

            if rclpy.ok():
                rclpy.shutdown()

        except Exception:
            pass


        self.root.destroy()


# ============================================================
# FUNCIÓN PRINCIPAL
# ============================================================

def main(args=None):

    # --------------------------------------------------------
    # Inicializamos ROS 2
    # --------------------------------------------------------

    rclpy.init(
        args=args
    )


    # --------------------------------------------------------
    # Creamos el nodo
    # --------------------------------------------------------

    node = TeleopNode()


    # --------------------------------------------------------
    # ROS 2 se ejecuta en un hilo separado.
    # --------------------------------------------------------

    ros_thread = threading.Thread(
        target=rclpy.spin,
        args=(node,),
        daemon=True
    )

    ros_thread.start()


    # --------------------------------------------------------
    # Ejecutamos la interfaz gráfica.
    # --------------------------------------------------------

    try:

        node.root.mainloop()

    except KeyboardInterrupt:

        pass

    finally:

        # ----------------------------------------------------
        # Si la interfaz todavía está abierta, cerramos
        # correctamente ROS.
        # ----------------------------------------------------

        if rclpy.ok():

            try:
                rclpy.shutdown()

            except Exception:
                pass


        try:
            node.destroy_node()

        except Exception:
            pass


# ============================================================
# EJECUCIÓN DIRECTA
# ============================================================

if __name__ == '__main__':
    main()
