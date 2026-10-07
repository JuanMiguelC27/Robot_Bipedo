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
#   6. Mostrar las matrices homogéneas T01, T02, T03, T04 y T05.
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
import os
import threading

import rclpy
from rclpy.node import Node

from robot_interfaces.msg import (
    RobotCommand, JointState, IKResult, IKJacobTarget
)
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

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

from robot_kinematics.kinem_invers_leg_mth import verificar_con_mth

from robot_kinematics.kinem_invers_leg_algebraico import (
    cinematica_inversa_pata_alg
)

from robot_kinematics.kinem_invers_leg_jacob import (
    cinematica_inversa_pata_jacob
)

from robot_kinematics.kinem_invers_leg_mth_Desacople import (
    cinematica_inversa_pata_des
)


# ------------------------------------------------------------
# POSICIÓN DE "HOME" PARA LA PESTAÑA DE CINEMÁTICA INVERSA (mm)
# ------------------------------------------------------------

HOME_X = 850.28
HOME_Y = 0.0
HOME_Z = -306.98   # pierna izquierda (q = 0)


# ============================================================
# TRAYECTORIA DESDE ARCHIVO (pestaña de IK algebraica)
# ============================================================
#
# El archivo trayectoria.txt vive en
# src/robot_kinematics/trayectorias/ y se instala con colcon en
# share/robot_kinematics/trayectorias/. Después de editarlo hay
# que volver a correr `colcon build`.
#
# Formato: una línea por punto "x, y, z" en mm (mismo sistema
# de coordenadas que los campos X, Y, Z de la pestaña). Se
# aceptan comas o espacios como separador (y corchetes o paréntesis
# alrededor de cada punto); las líneas vacías y
# lo que va después de '#' se ignoran.
#
# ============================================================

TRAJECTORY_FILE_NAME = 'trayectoria.txt'


def _trajectory_file_path():

    from ament_index_python.packages import get_package_share_directory

    return os.path.join(
        get_package_share_directory('robot_kinematics'),
        'trayectorias',
        TRAJECTORY_FILE_NAME
    )


def _load_trajectory_file(path):

    # Devuelve una lista de (n_linea, x, y, z). Lanza ValueError
    # con el número de línea si alguna no tiene el formato.

    points = []

    with open(path, 'r', encoding='utf-8') as f:

        for line_number, line in enumerate(f, start=1):

            line = line.split('#', 1)[0].strip()

            if not line:
                continue

            # Se aceptan también puntos entre corchetes o
            # paréntesis: "[x, y, z]" o "(x, y, z)".
            for char in '[](),':
                line = line.replace(char, ' ')

            values = line.split()

            if len(values) != 3:
                raise ValueError(
                    f"línea {line_number}: se esperaban 3 valores "
                    f"(x, y, z), hay {len(values)}."
                )

            try:
                x, y, z = (float(v) for v in values)
            except ValueError:
                raise ValueError(
                    f"línea {line_number}: valor no numérico."
                )

            points.append((line_number, x, y, z))

    return points


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
            0,    # Hip Roll
            -90.0,    # Hip Pitch
            -90.0     # Knee Pitch
        ]

        self.upper_deg = [
            90.0,     # Hip Roll
            90.0,     # Hip Pitch
            90.0      # Knee Pitch
        ]


        # ----------------------------------------------------
        # OFFSETS DE LOS SERVOS REVISAR
        # ----------------------------------------------------

        self.servo_offset_deg = [
            0,    # Hip Roll
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

        self.selected_mth = 'T05'


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
        # CINEMÁTICA INVERSA (vía ik_node, por tópicos)
        # ====================================================

        self.ik_target_pub = self.create_publisher(
            Point,
            '/robot/ik_target',
            10
        )

        self.ik_result_sub = self.create_subscription(
            IKResult,
            '/robot/ik_result',
            self.on_ik_result,
            10
        )

        # Resultado pendiente de aplicar en la GUI. Se llena en
        # on_ik_result (hilo de ROS) y se consume en refresh()
        # (hilo de Tkinter), igual que current_deg/real_deg.

        self.pending_ik_result = None


        # ----------------------------------------------------
        # Trayectorias desde archivo, una por pestaña de IK
        # ('alg', 'jacob', 'des'). Cada entrada guarda sus
        # widgets (los llena build_trajectory_frame), la lista
        # de puntos (x, y, z) en mm a visitar, el índice del
        # punto actual, si está corriendo, y el id del
        # root.after() pendiente (para cancelarlo con "Parar").
        # ----------------------------------------------------

        self.trajectories = {}


        # ----------------------------------------------------
        # RASTRO DEL RECORRIDO (verificación visual en RViz)
        #
        # Cada vez que un objetivo de cinemática inversa es
        # alcanzable y se aplica (en cualquiera de las 3
        # pestañas de IK), se agrega el punto a esta lista y se
        # publica como un Marker tipo LINE_STRIP, en el mismo
        # frame que usa la cinemática (Base_link), para poder
        # ver en RViz el camino real que fue recorriendo el pie.
        # ----------------------------------------------------

        self.trail_points = []

        self.trail_pub = self.create_publisher(
            Marker,
            '/robot/ik_trail',
            10
        )


        # ====================================================
        # CINEMÁTICA INVERSA - MÉTODO DEL JACOBIANO
        # (vía ik_jacob_node, por tópicos)
        # ====================================================

        self.ik_jacob_target_pub = self.create_publisher(
            IKJacobTarget,
            '/robot/ik_jacob_target',
            10
        )

        self.ik_jacob_result_sub = self.create_subscription(
            IKResult,
            '/robot/ik_jacob_result',
            self.on_ik_jacob_result,
            10
        )

        self.pending_ik_jacob_result = None


        # ====================================================
        # CINEMÁTICA INVERSA - MÉTODO DE DESACOPLE
        # (vía ik_des_node, por tópicos)
        # ====================================================

        self.ik_des_target_pub = self.create_publisher(
            Point,
            '/robot/ik_des_target',
            10
        )

        self.ik_des_result_sub = self.create_subscription(
            IKResult,
            '/robot/ik_des_result',
            self.on_ik_des_result,
            10
        )

        self.pending_ik_des_result = None


        # ====================================================
        # VERIFICACIÓN MTH (matriz de transformación homogénea)
        #
        # No usa tópicos: es un cálculo cerrado (sin iteraciones)
        # que se llama directamente desde la GUI, igual que ya se
        # hace con la cinemática directa de kinem_leg_gen. Guarda
        # el último objetivo (x, y, z) pedido en cada pestaña de
        # IK para poder comparar contra la posición que realmente
        # produce la pose calculada.
        # ====================================================

        self.last_ik_target = None
        self.last_ik_jacob_target = None
        self.last_ik_des_target = None


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
            text="Control de articulaciones, cinemática directa e inversa",
            style='Subtitle.TLabel'
        )

        subtitle_label.pack(
            pady=(0, 8)
        )


        # ====================================================
        # PESTAÑAS: CINEMÁTICA DIRECTA / CINEMÁTICA INVERSA
        # ====================================================

        notebook = ttk.Notebook(
            main_frame
        )

        notebook.pack(
            fill='both',
            expand=True
        )

        tab_fk = ttk.Frame(
            notebook,
            padding=0
        )

        tab_ik = ttk.Frame(
            notebook,
            padding=8
        )

        tab_ik_jacob = ttk.Frame(
            notebook,
            padding=8
        )

        tab_ik_des = ttk.Frame(
            notebook,
            padding=8
        )

        notebook.add(
            tab_fk,
            text="Cinemática directa"
        )

        notebook.add(
            tab_ik,
            text="Cinemática inversa"
        )

        notebook.add(
            tab_ik_jacob,
            text="Cinemática inversa (Jacobiano)"
        )

        notebook.add(
            tab_ik_des,
            text="Cinemática inversa (Desacople)"
        )


        # ====================================================
        # SECCIÓN DE ARTICULACIONES
        # ====================================================

        joints_frame = ttk.LabelFrame(
            tab_fk,
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
            tab_fk,
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
            tab_fk,
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
                'T04',
                'T05'
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
        # PESTAÑA DE CINEMÁTICA INVERSA
        # ====================================================

        ik_coords_frame = ttk.LabelFrame(
            tab_ik,
            text="Coordenada objetivo (mm)",
            padding=8,
            style='Section.TLabelframe'
        )

        ik_coords_frame.pack(
            fill='x',
            pady=(0, 8)
        )

        self.ik_entries = {}

        for i, axis in enumerate(('X', 'Y', 'Z')):

            label = ttk.Label(
                ik_coords_frame,
                text=f"{axis}:"
            )

            label.grid(
                row=0,
                column=i * 2,
                padx=(5, 3),
                pady=5
            )

            entry = tk.Entry(
                ik_coords_frame,
                width=8,
                justify='center'
            )

            entry.insert(
                0,
                "0.0"
            )

            entry.grid(
                row=0,
                column=i * 2 + 1,
                padx=(0, 10),
                pady=5
            )

            self.ik_entries[axis] = entry

        ik_send_button = ttk.Button(
            ik_coords_frame,
            text="Calcular y enviar",
            command=self.send_ik_target
        )

        ik_send_button.grid(
            row=1,
            column=0,
            columnspan=3,
            pady=(5, 0)
        )

        ik_home_button = tk.Button(
            ik_coords_frame,
            text="Home",
            bg='#ffdddd',
            command=self.go_home
        )

        ik_home_button.grid(
            row=1,
            column=3,
            columnspan=3,
            pady=(5, 0)
        )


        # ----------------------------------------------------
        # RESULTADO (q1, q2, q3)
        # ----------------------------------------------------

        ik_result_frame = ttk.LabelFrame(
            tab_ik,
            text="Resultado",
            padding=8,
            style='Section.TLabelframe'
        )

        ik_result_frame.pack(
            fill='x',
            pady=(0, 8)
        )

        self.ik_q_labels = []

        for i, name in enumerate(
            ('q1 (Hip Roll)', 'q2 (Hip Pitch)', 'q3 (Knee)')
        ):

            label = ttk.Label(
                ik_result_frame,
                text=f"{name}:",
                width=16
            )

            label.grid(
                row=i,
                column=0,
                padx=(5, 5),
                pady=3,
                sticky='w'
            )

            value_label = ttk.Label(
                ik_result_frame,
                text="--",
                width=10
            )

            value_label.grid(
                row=i,
                column=1,
                padx=(0, 5),
                pady=3,
                sticky='w'
            )

            self.ik_q_labels.append(
                value_label
            )


        # ----------------------------------------------------
        # APLICAR A HARDWARE (ESP32 real)
        #
        # Envía por /servo_commands la última consigna calculada
        # (self.target_deg, ya sincronizada con el resultado de
        # esta pestaña). La previsualización en RViz ya ocurre
        # sola al calcular, por /robot/command.
        # ----------------------------------------------------

        ik_hardware_button = tk.Button(
            tab_ik,
            text="Aplicar a motores (HARDWARE)",
            bg='#ffdddd',
            command=lambda: self.apply_to_hardware(self.set_ik_status)
        )

        ik_hardware_button.pack(
            pady=(0, 8)
        )


        # ----------------------------------------------------
        # TRAYECTORIA DESDE ARCHIVO (trayectoria.txt)
        # ----------------------------------------------------

        self.build_trajectory_frame(
            tab_ik,
            'alg',
            self.ik_entries,
            self.send_ik_target,
            self.set_ik_status
        )


        # ----------------------------------------------------
        # VERIFICACIÓN MTH DEL RESULTADO
        # ----------------------------------------------------

        self.ik_mth = self.build_mth_verification(tab_ik)


        # ----------------------------------------------------
        # MENSAJE DE ESTADO (pestaña IK)
        # ----------------------------------------------------

        self.ik_status_label = tk.Label(
            tab_ik,
            text="Ingrese una coordenada y presione \"Calcular y enviar\".",
            font=('Arial', 10, 'bold'),
            wraplength=380,
            justify='center'
        )

        self.ik_status_label.pack(
            pady=(5, 0)
        )


        # ====================================================
        # PESTAÑA DE CINEMÁTICA INVERSA (JACOBIANO)
        # ====================================================

        ikj_coords_frame = ttk.LabelFrame(
            tab_ik_jacob,
            text="Coordenada objetivo (mm)",
            padding=8,
            style='Section.TLabelframe'
        )

        ikj_coords_frame.pack(
            fill='x',
            pady=(0, 8)
        )

        self.ikj_entries = {}

        for i, axis in enumerate(('X', 'Y', 'Z')):

            label = ttk.Label(
                ikj_coords_frame,
                text=f"{axis}:"
            )

            label.grid(
                row=0,
                column=i * 2,
                padx=(5, 3),
                pady=5
            )

            entry = tk.Entry(
                ikj_coords_frame,
                width=8,
                justify='center'
            )

            entry.insert(
                0,
                "0.0"
            )

            entry.grid(
                row=0,
                column=i * 2 + 1,
                padx=(0, 10),
                pady=5
            )

            self.ikj_entries[axis] = entry

        ikj_send_button = ttk.Button(
            tab_ik_jacob,
            text="Calcular y enviar",
            command=self.send_ik_jacob_target
        )

        ikj_send_button.pack(
            pady=(0, 4)
        )

        ikj_home_button = tk.Button(
            tab_ik_jacob,
            text="Home",
            bg='#ffdddd',
            command=self.go_home_jacob
        )

        ikj_home_button.pack(
            pady=(0, 8)
        )


        # ----------------------------------------------------
        # RESULTADO (q1, q2, q3)
        # ----------------------------------------------------

        ikj_result_frame = ttk.LabelFrame(
            tab_ik_jacob,
            text="Resultado",
            padding=8,
            style='Section.TLabelframe'
        )

        ikj_result_frame.pack(
            fill='x',
            pady=(0, 8)
        )

        self.ikj_q_labels = []

        for i, name in enumerate(
            ('q1 (Hip Roll)', 'q2 (Hip Pitch)', 'q3 (Knee)')
        ):

            label = ttk.Label(
                ikj_result_frame,
                text=f"{name}:",
                width=16
            )

            label.grid(
                row=i,
                column=0,
                padx=(5, 5),
                pady=3,
                sticky='w'
            )

            value_label = ttk.Label(
                ikj_result_frame,
                text="--",
                width=10
            )

            value_label.grid(
                row=i,
                column=1,
                padx=(0, 5),
                pady=3,
                sticky='w'
            )

            self.ikj_q_labels.append(
                value_label
            )


        # ----------------------------------------------------
        # APLICAR A HARDWARE (ESP32 real)
        # ----------------------------------------------------

        ikj_hardware_button = tk.Button(
            tab_ik_jacob,
            text="Aplicar a motores (HARDWARE)",
            bg='#ffdddd',
            command=lambda: self.apply_to_hardware(self.set_ik_jacob_status)
        )

        ikj_hardware_button.pack(
            pady=(0, 8)
        )


        # ----------------------------------------------------
        # TRAYECTORIA DESDE ARCHIVO (trayectoria.txt)
        # ----------------------------------------------------

        self.build_trajectory_frame(
            tab_ik_jacob,
            'jacob',
            self.ikj_entries,
            self.send_ik_jacob_target,
            self.set_ik_jacob_status
        )


        # ----------------------------------------------------
        # VERIFICACIÓN MTH DEL RESULTADO
        # ----------------------------------------------------

        self.ikj_mth = self.build_mth_verification(tab_ik_jacob)


        # ----------------------------------------------------
        # MENSAJE DE ESTADO (pestaña Jacobiano)
        # ----------------------------------------------------

        self.ikj_status_label = tk.Label(
            tab_ik_jacob,
            text="Ingrese objetivo y semilla, luego \"Calcular y enviar\".",
            font=('Arial', 10, 'bold'),
            wraplength=380,
            justify='center'
        )

        self.ikj_status_label.pack(
            pady=(5, 0)
        )


        # ====================================================
        # PESTAÑA DE CINEMÁTICA INVERSA (DESACOPLE)
        # ====================================================

        ikd_coords_frame = ttk.LabelFrame(
            tab_ik_des,
            text="Coordenada objetivo (mm)",
            padding=8,
            style='Section.TLabelframe'
        )

        ikd_coords_frame.pack(
            fill='x',
            pady=(0, 8)
        )

        self.ikd_entries = {}

        for i, axis in enumerate(('X', 'Y', 'Z')):

            label = ttk.Label(
                ikd_coords_frame,
                text=f"{axis}:"
            )

            label.grid(
                row=0,
                column=i * 2,
                padx=(5, 3),
                pady=5
            )

            entry = tk.Entry(
                ikd_coords_frame,
                width=8,
                justify='center'
            )

            entry.insert(
                0,
                "0.0"
            )

            entry.grid(
                row=0,
                column=i * 2 + 1,
                padx=(0, 10),
                pady=5
            )

            self.ikd_entries[axis] = entry

        ikd_send_button = ttk.Button(
            ikd_coords_frame,
            text="Calcular y enviar",
            command=self.send_ik_des_target
        )

        ikd_send_button.grid(
            row=1,
            column=0,
            columnspan=3,
            pady=(5, 0)
        )

        ikd_home_button = tk.Button(
            ikd_coords_frame,
            text="Home",
            bg='#ffdddd',
            command=self.go_home_des
        )

        ikd_home_button.grid(
            row=1,
            column=3,
            columnspan=3,
            pady=(5, 0)
        )


        # ----------------------------------------------------
        # RESULTADO (q1, q2, q3)
        # ----------------------------------------------------

        ikd_result_frame = ttk.LabelFrame(
            tab_ik_des,
            text="Resultado",
            padding=8,
            style='Section.TLabelframe'
        )

        ikd_result_frame.pack(
            fill='x',
            pady=(0, 8)
        )

        self.ikd_q_labels = []

        for i, name in enumerate(
            ('q1 (Hip Roll)', 'q2 (Hip Pitch)', 'q3 (Knee)')
        ):

            label = ttk.Label(
                ikd_result_frame,
                text=f"{name}:",
                width=16
            )

            label.grid(
                row=i,
                column=0,
                padx=(5, 5),
                pady=3,
                sticky='w'
            )

            value_label = ttk.Label(
                ikd_result_frame,
                text="--",
                width=10
            )

            value_label.grid(
                row=i,
                column=1,
                padx=(0, 5),
                pady=3,
                sticky='w'
            )

            self.ikd_q_labels.append(
                value_label
            )


        # ----------------------------------------------------
        # APLICAR A HARDWARE (ESP32 real)
        # ----------------------------------------------------

        ikd_hardware_button = tk.Button(
            tab_ik_des,
            text="Aplicar a motores (HARDWARE)",
            bg='#ffdddd',
            command=lambda: self.apply_to_hardware(self.set_ik_des_status)
        )

        ikd_hardware_button.pack(
            pady=(0, 8)
        )


        # ----------------------------------------------------
        # TRAYECTORIA DESDE ARCHIVO (trayectoria.txt)
        # ----------------------------------------------------

        self.build_trajectory_frame(
            tab_ik_des,
            'des',
            self.ikd_entries,
            self.send_ik_des_target,
            self.set_ik_des_status
        )


        # ----------------------------------------------------
        # VERIFICACIÓN MTH DEL RESULTADO
        # ----------------------------------------------------

        self.ikd_mth = self.build_mth_verification(tab_ik_des)


        # ----------------------------------------------------
        # MENSAJE DE ESTADO (pestaña Desacople)
        # ----------------------------------------------------

        self.ikd_status_label = tk.Label(
            tab_ik_des,
            text="Ingrese una coordenada y presione \"Calcular y enviar\".",
            font=('Arial', 10, 'bold'),
            wraplength=380,
            justify='center'
        )

        self.ikd_status_label.pack(
            pady=(5, 0)
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

    def apply_to_hardware(self, set_status=None):

        # Envía al hardware real la última consigna calculada,
        # sea por los sliders (Cinemática directa) o por el
        # resultado de cualquiera de las 3 pestañas de cinemática
        # inversa: todas actualizan self.target_deg al llegar.
        #
        # set_status permite avisar en la pestaña desde la que se
        # apretó el botón (si no se pasa, se usa la de Cinemática
        # directa, por compatibilidad con el botón original).

        if set_status is None:
            set_status = self.set_status

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

        set_status(
            "✓ Enviado al motor real (última consigna calculada).",
            "green"
        )


    # ========================================================
    # CINEMÁTICA INVERSA - ENVIAR OBJETIVO
    # ========================================================

    def send_ik_target(self):

        try:
            x = float(self.ik_entries['X'].get())
            y = float(self.ik_entries['Y'].get())
            z = float(self.ik_entries['Z'].get())

        except ValueError:

            self.set_ik_status(
                "Error: X, Y, Z deben ser valores numéricos.",
                "red"
            )

            return

        self.last_ik_target = (x, y, z)

        point = Point()
        point.x = x
        point.y = y
        point.z = z

        self.ik_target_pub.publish(point)

        self.set_ik_status(
            "Calculando...",
            "gray"
        )


    # ========================================================
    # CINEMÁTICA INVERSA - IR A HOME
    # ========================================================

    def go_home(self):

        self.ik_entries['X'].delete(0, tk.END)
        self.ik_entries['X'].insert(0, f"{HOME_X}")

        self.ik_entries['Y'].delete(0, tk.END)
        self.ik_entries['Y'].insert(0, f"{HOME_Y}")

        self.ik_entries['Z'].delete(0, tk.END)
        self.ik_entries['Z'].insert(0, f"{HOME_Z}")

        self.send_ik_target()


    # ========================================================
    # TRAYECTORIA DESDE ARCHIVO - WIDGETS DE UNA PESTAÑA
    #
    # Se llama una vez por cada pestaña de cinemática inversa.
    # Lee los puntos (x, y, z) de trayectoria.txt, verifica que
    # todos tengan solución con el método de esa pestaña y los
    # va enviando uno por uno, con una pausa entre cada uno. Por
    # defecto solo se mueve el modelo en RViz; con la casilla
    # marcada también se envía cada punto a los motores reales.
    # ========================================================

    def build_trajectory_frame(
        self, parent, key, entries, send_target, set_status
    ):

        traj_frame = ttk.LabelFrame(
            parent,
            text=f"Trayectoria desde archivo ({TRAJECTORY_FILE_NAME})",
            padding=8,
            style='Section.TLabelframe'
        )

        traj_frame.pack(
            fill='x',
            pady=(0, 8)
        )

        ttk.Label(
            traj_frame,
            text="Intervalo (ms):"
        ).grid(
            row=0,
            column=0,
            padx=(5, 3),
            pady=5
        )

        interval_entry = tk.Entry(
            traj_frame,
            width=8,
            justify='center'
        )

        interval_entry.insert(0, "150")

        interval_entry.grid(
            row=0,
            column=1,
            padx=(0, 10),
            pady=5
        )

        hardware_var = tk.BooleanVar(value=False)

        tk.Checkbutton(
            traj_frame,
            text="Enviar también a motores",
            variable=hardware_var
        ).grid(
            row=0,
            column=2,
            columnspan=2,
            padx=(5, 3),
            pady=5
        )

        ttk.Button(
            traj_frame,
            text="Seguir trayectoria",
            command=lambda: self.start_file_trajectory(key)
        ).grid(
            row=1,
            column=0,
            columnspan=2,
            pady=(5, 0)
        )

        tk.Button(
            traj_frame,
            text="Parar",
            bg='#ffdddd',
            command=lambda: self.stop_file_trajectory(key)
        ).grid(
            row=1,
            column=2,
            columnspan=2,
            pady=(5, 0)
        )

        tk.Button(
            traj_frame,
            text="Limpiar rastro (RViz)",
            bg='#ddeeff',
            command=self.clear_trail
        ).grid(
            row=2,
            column=0,
            columnspan=4,
            pady=(5, 0)
        )

        self.trajectories[key] = {
            'entries': entries,
            'send_target': send_target,
            'set_status': set_status,
            'interval_entry': interval_entry,
            'hardware_var': hardware_var,
            'points': [],
            'index': 0,
            'running': False,
            'after_id': None,
            'interval_ms': 150,
        }


    # ========================================================
    # TRAYECTORIA DESDE ARCHIVO - VALIDAR TODOS LOS PUNTOS
    #
    # Resuelve cada punto con el mismo método de la pestaña.
    # Para el Jacobiano se encadena la semilla igual que en la
    # ejecución real: se parte de la pose actual y cada punto
    # usa como semilla la solución del anterior. Devuelve el
    # primer punto sin solución, o None si todos la tienen.
    # ========================================================

    def _first_unreachable_point(self, key, points):

        seed_deg = list(self.target_deg)

        for line_number, x, y, z in points:

            if key == 'jacob':
                q1, q2, q3, ok = cinematica_inversa_pata_jacob(
                    x, y, z, *seed_deg
                )
            elif key == 'des':
                q1, q2, q3, ok = cinematica_inversa_pata_des(x, y, z)
            else:
                q1, q2, q3, ok = cinematica_inversa_pata_alg(x, y, z)

            if not ok:
                return line_number, x, y, z

            seed_deg = [math.degrees(q) for q in (q1, q2, q3)]

        return None


    # ========================================================
    # TRAYECTORIA DESDE ARCHIVO - CARGAR, VALIDAR Y LANZAR
    # ========================================================

    def start_file_trajectory(self, key):

        traj = self.trajectories[key]
        set_status = traj['set_status']

        try:
            interval_ms = int(float(traj['interval_entry'].get()))

        except ValueError:

            set_status(
                "Error: el intervalo debe ser numérico.",
                "red"
            )

            return

        if interval_ms < 20:
            interval_ms = 20

        try:
            path = _trajectory_file_path()
            points = _load_trajectory_file(path)

        except FileNotFoundError:

            set_status(
                f"Error: no se encontró {TRAJECTORY_FILE_NAME} "
                f"(¿se corrió colcon build?).",
                "red"
            )

            return

        except Exception as error:

            set_status(
                f"Error en {TRAJECTORY_FILE_NAME}: {error}",
                "red"
            )

            return

        if not points:

            set_status(
                f"Error: {TRAJECTORY_FILE_NAME} no tiene puntos.",
                "red"
            )

            return

        # Antes de mover nada se verifica que todos los puntos
        # tengan solución, para no quedar a mitad de camino.

        unreachable = self._first_unreachable_point(key, points)

        if unreachable is not None:

            line_number, x, y, z = unreachable

            set_status(
                f"✗ Línea {line_number}: ({x:.1f}, {y:.1f}, "
                f"{z:.1f}) no alcanzable. No se ejecutó.",
                "red"
            )

            return

        # Solo una trayectoria a la vez: se cancela cualquiera
        # que siga pendiente (en esta u otra pestaña).

        for other_key in self.trajectories:
            self.stop_file_trajectory(other_key)

        traj['points'] = [(x, y, z) for _, x, y, z in points]
        traj['index'] = 0
        traj['running'] = True
        traj['interval_ms'] = interval_ms

        set_status(
            f"Siguiendo trayectoria... (0/{len(traj['points'])})",
            "blue"
        )

        self._file_trajectory_step(key)


    # ========================================================
    # TRAYECTORIA DESDE ARCHIVO - UN PASO
    # (encadenado solo, vía root.after; no bloquea la GUI)
    # ========================================================

    def _file_trajectory_step(self, key):

        traj = self.trajectories[key]

        if not traj['running']:
            return

        if traj['index'] >= len(traj['points']):

            traj['running'] = False

            traj['set_status'](
                "✓ Trayectoria completa.",
                "green"
            )

            return

        x, y, z = traj['points'][traj['index']]

        entries = traj['entries']

        entries['X'].delete(0, tk.END)
        entries['X'].insert(0, f"{x:.2f}")

        entries['Y'].delete(0, tk.END)
        entries['Y'].insert(0, f"{y:.2f}")

        entries['Z'].delete(0, tk.END)
        entries['Z'].insert(0, f"{z:.2f}")

        traj['send_target']()

        traj['index'] += 1

        traj['set_status'](
            f"Siguiendo trayectoria... "
            f"({traj['index']}/{len(traj['points'])})",
            "blue"
        )

        traj['after_id'] = self.root.after(
            traj['interval_ms'],
            lambda: self._file_trajectory_step(key)
        )


    # ========================================================
    # TRAYECTORIA DESDE ARCHIVO - PARAR
    # (el robot se queda en el último punto alcanzado)
    # ========================================================

    def stop_file_trajectory(self, key):

        traj = self.trajectories[key]

        was_running = traj['running']

        traj['running'] = False

        if traj['after_id'] is not None:

            try:
                self.root.after_cancel(traj['after_id'])
            except Exception:
                pass

            traj['after_id'] = None

        if was_running:

            traj['set_status'](
                f"Trayectoria detenida en el punto "
                f"{traj['index']}/{len(traj['points'])}.",
                "orange"
            )


    # ========================================================
    # TRAYECTORIA DESDE ARCHIVO - TRAS UN RESULTADO DE IK
    #
    # Se llama desde apply_*_result cuando el punto fue
    # alcanzable. Si la pestaña está siguiendo una trayectoria,
    # envía el punto a los motores (si la casilla está marcada)
    # y devuelve True para que no se pise el mensaje de
    # progreso con "Movimiento realizado".
    # ========================================================

    def _on_trajectory_ik_result(self, key):

        traj = self.trajectories.get(key)

        if traj is None or not traj['running']:
            return False

        if traj['hardware_var'].get():
            self.apply_to_hardware(lambda *_: None)

        return True


    # ========================================================
    # RASTRO DEL RECORRIDO - AGREGAR PUNTO Y PUBLICAR
    #
    # x, y, z llegan en milímetros (igual que el resto de la
    # cinemática); Marker/RViz trabajan en metros, por eso se
    # dividen entre 1000 acá.
    # ========================================================

    def add_trail_point(self, x, y, z):

        point = Point()
        point.x = x / 1000.0
        point.y = y / 1000.0
        point.z = z / 1000.0

        self.trail_points.append(point)

        self.publish_trail()


    def publish_trail(self):

        marker = Marker()

        marker.header.frame_id = 'Base_link'
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = 'ik_trail'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        # ----------------------------------------------------
        # Los puntos están en el marco base de la DH
        # (kinem_leg_gen), que no coincide con Base_link:
        #
        #   X DH (largo de la pierna) -> -Y Base_link
        #   Y DH (adelante/atrás)     -> +X Base_link
        #   Z DH (lateral)            -> +Z Base_link
        #
        # Es una rotación de -90° en Z más un desplazamiento
        # del origen (medido contra el URDF, en metros). Se
        # aplica con la pose del Marker, así los puntos
        # quedan tal cual vienen de la cinemática.
        # ----------------------------------------------------

        if self.leg_side == 'left':
            offset = (0.0950, -0.0071, 0.0050)
        else:
            offset = (0.0816, -0.0070, -0.0056)

        marker.pose.position.x = offset[0]
        marker.pose.position.y = offset[1]
        marker.pose.position.z = offset[2]

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = -math.sqrt(0.5)
        marker.pose.orientation.w = math.sqrt(0.5)

        marker.scale.x = 0.005

        marker.color.r = 1.0
        marker.color.g = 0.1
        marker.color.b = 0.5
        marker.color.a = 1.0

        marker.points = list(self.trail_points)

        self.trail_pub.publish(marker)


    # ========================================================
    # RASTRO DEL RECORRIDO - LIMPIAR
    # ========================================================

    def clear_trail(self):

        self.trail_points = []

        self.publish_trail()


    # ========================================================
    # CINEMÁTICA INVERSA - RECEPCIÓN DEL RESULTADO
    # (hilo de ROS; solo guarda el dato, ver refresh())
    # ========================================================

    def on_ik_result(self, msg):

        self.pending_ik_result = msg


    # ========================================================
    # CINEMÁTICA INVERSA - APLICAR RESULTADO
    # (hilo de Tkinter, llamado desde refresh())
    # ========================================================

    def apply_ik_result(self, result):

        if not result.reachable:

            for label in self.ik_q_labels:
                label.config(text="--")

            self.set_ik_status(
                "✗ Posición no alcanzable.",
                "red"
            )

            self.reset_mth_verification(self.ik_mth)

            return

        q_deg = [math.degrees(q) for q in result.position]

        for label, value in zip(self.ik_q_labels, q_deg):
            label.config(text=f"{value:.2f}°")


        # ----------------------------------------------------
        # Movemos el modelo en RViz por el mismo camino que la
        # pestaña de cinemática directa (kinematics_node ->
        # control_node -> robot_state_publisher).
        # ----------------------------------------------------

        command_msg = RobotCommand()
        command_msg.position = list(result.position)
        self.command_pub.publish(command_msg)


        # ----------------------------------------------------
        # Sincronizamos sliders/entradas/matriz de la pestaña
        # de cinemática directa con la nueva pose, para que
        # ambas pestañas muestren siempre el mismo estado.
        # ----------------------------------------------------

        for i in range(min(self.num_joints, len(q_deg))):

            self.target_deg[i] = q_deg[i]

            self.sliders[i].set(q_deg[i])

            self.slider_labels[i].config(
                text=f"{q_deg[i]:.1f}°"
            )

            self.entries[i].delete(0, tk.END)
            self.entries[i].insert(0, f"{q_deg[i]:.1f}")

        self.update_kinematics()

        if self.last_ik_target is not None:
            x_obj, y_obj, z_obj = self.last_ik_target
            self.update_mth_verification(
                self.ik_mth,
                result.position[0], result.position[1], result.position[2],
                x_obj, y_obj, z_obj
            )
            self.add_trail_point(x_obj, y_obj, z_obj)

        if self._on_trajectory_ik_result('alg'):
            return

        self.set_ik_status(
            "✓ Movimiento realizado.",
            "green"
        )


    # ========================================================
    # CINEMÁTICA INVERSA - MENSAJE DE ESTADO
    # ========================================================

    def set_ik_status(self, message, color):

        self.ik_status_label.config(
            text=message,
            foreground=color
        )


    # ========================================================
    # CINEMÁTICA INVERSA (JACOBIANO) - ENVIAR OBJETIVO
    # ========================================================

    def send_ik_jacob_target(self):

        try:
            x = float(self.ikj_entries['X'].get())
            y = float(self.ikj_entries['Y'].get())
            z = float(self.ikj_entries['Z'].get())

        except ValueError:

            self.set_ik_jacob_status(
                "Error: X, Y, Z deben ser valores numéricos.",
                "red"
            )

            return

        self.last_ik_jacob_target = (x, y, z)

        # La semilla no se pide al usuario: se toma la pose
        # articular actual (self.target_deg), que es la misma
        # que se ve en la pestaña de cinemática directa.

        target = IKJacobTarget()
        target.x = x
        target.y = y
        target.z = z
        target.q1_seed = self.target_deg[0]
        target.q2_seed = self.target_deg[1]
        target.q3_seed = self.target_deg[2]

        self.ik_jacob_target_pub.publish(target)

        self.set_ik_jacob_status(
            "Calculando...",
            "gray"
        )


    # ========================================================
    # CINEMÁTICA INVERSA (JACOBIANO) - IR A HOME
    # ========================================================

    def go_home_jacob(self):

        self.ikj_entries['X'].delete(0, tk.END)
        self.ikj_entries['X'].insert(0, f"{HOME_X}")

        self.ikj_entries['Y'].delete(0, tk.END)
        self.ikj_entries['Y'].insert(0, f"{HOME_Y}")

        self.ikj_entries['Z'].delete(0, tk.END)
        self.ikj_entries['Z'].insert(0, f"{HOME_Z}")

        self.send_ik_jacob_target()


    # ========================================================
    # CINEMÁTICA INVERSA (JACOBIANO) - RECEPCIÓN DEL RESULTADO
    # (hilo de ROS; solo guarda el dato, ver refresh())
    # ========================================================

    def on_ik_jacob_result(self, msg):

        self.pending_ik_jacob_result = msg


    # ========================================================
    # CINEMÁTICA INVERSA (JACOBIANO) - APLICAR RESULTADO
    # (hilo de Tkinter, llamado desde refresh())
    # ========================================================

    def apply_ik_jacob_result(self, result):

        if not result.reachable:

            for label in self.ikj_q_labels:
                label.config(text="--")

            self.set_ik_jacob_status(
                "✗ No convergió / posición no alcanzable.",
                "red"
            )

            self.reset_mth_verification(self.ikj_mth)

            return

        q_deg = [math.degrees(q) for q in result.position]

        for label, value in zip(self.ikj_q_labels, q_deg):
            label.config(text=f"{value:.2f}°")


        # ----------------------------------------------------
        # Movemos el modelo en RViz por el mismo camino que las
        # demás pestañas (kinematics_node -> control_node ->
        # robot_state_publisher).
        # ----------------------------------------------------

        command_msg = RobotCommand()
        command_msg.position = list(result.position)
        self.command_pub.publish(command_msg)


        # ----------------------------------------------------
        # Sincronizamos sliders/entradas/matriz de la pestaña
        # de cinemática directa con la nueva pose.
        # ----------------------------------------------------

        for i in range(min(self.num_joints, len(q_deg))):

            self.target_deg[i] = q_deg[i]

            self.sliders[i].set(q_deg[i])

            self.slider_labels[i].config(
                text=f"{q_deg[i]:.1f}°"
            )

            self.entries[i].delete(0, tk.END)
            self.entries[i].insert(0, f"{q_deg[i]:.1f}")

        self.update_kinematics()

        if self.last_ik_jacob_target is not None:
            x_obj, y_obj, z_obj = self.last_ik_jacob_target
            self.update_mth_verification(
                self.ikj_mth,
                result.position[0], result.position[1], result.position[2],
                x_obj, y_obj, z_obj
            )
            self.add_trail_point(x_obj, y_obj, z_obj)

        if self._on_trajectory_ik_result('jacob'):
            return

        self.set_ik_jacob_status(
            "✓ Movimiento realizado.",
            "green"
        )


    # ========================================================
    # CINEMÁTICA INVERSA (JACOBIANO) - MENSAJE DE ESTADO
    # ========================================================

    def set_ik_jacob_status(self, message, color):

        self.ikj_status_label.config(
            text=message,
            foreground=color
        )


    # ========================================================
    # CINEMÁTICA INVERSA (DESACOPLE) - ENVIAR OBJETIVO
    # ========================================================

    def send_ik_des_target(self):

        try:
            x = float(self.ikd_entries['X'].get())
            y = float(self.ikd_entries['Y'].get())
            z = float(self.ikd_entries['Z'].get())

        except ValueError:

            self.set_ik_des_status(
                "Error: X, Y, Z deben ser valores numéricos.",
                "red"
            )

            return

        self.last_ik_des_target = (x, y, z)

        point = Point()
        point.x = x
        point.y = y
        point.z = z

        self.ik_des_target_pub.publish(point)

        self.set_ik_des_status(
            "Calculando...",
            "gray"
        )


    # ========================================================
    # CINEMÁTICA INVERSA (DESACOPLE) - IR A HOME
    # ========================================================

    def go_home_des(self):

        self.ikd_entries['X'].delete(0, tk.END)
        self.ikd_entries['X'].insert(0, f"{HOME_X}")

        self.ikd_entries['Y'].delete(0, tk.END)
        self.ikd_entries['Y'].insert(0, f"{HOME_Y}")

        self.ikd_entries['Z'].delete(0, tk.END)
        self.ikd_entries['Z'].insert(0, f"{HOME_Z}")

        self.send_ik_des_target()


    # ========================================================
    # CINEMÁTICA INVERSA (DESACOPLE) - RECEPCIÓN DEL RESULTADO
    # (hilo de ROS; solo guarda el dato, ver refresh())
    # ========================================================

    def on_ik_des_result(self, msg):

        self.pending_ik_des_result = msg


    # ========================================================
    # CINEMÁTICA INVERSA (DESACOPLE) - APLICAR RESULTADO
    # (hilo de Tkinter, llamado desde refresh())
    # ========================================================

    def apply_ik_des_result(self, result):

        if not result.reachable:

            for label in self.ikd_q_labels:
                label.config(text="--")

            self.set_ik_des_status(
                "✗ Posición no alcanzable.",
                "red"
            )

            self.reset_mth_verification(self.ikd_mth)

            return

        q_deg = [math.degrees(q) for q in result.position]

        for label, value in zip(self.ikd_q_labels, q_deg):
            label.config(text=f"{value:.2f}°")


        # ----------------------------------------------------
        # Movemos el modelo en RViz por el mismo camino que las
        # demás pestañas (kinematics_node -> control_node ->
        # robot_state_publisher).
        # ----------------------------------------------------

        command_msg = RobotCommand()
        command_msg.position = list(result.position)
        self.command_pub.publish(command_msg)


        # ----------------------------------------------------
        # Sincronizamos sliders/entradas/matriz de la pestaña
        # de cinemática directa con la nueva pose.
        # ----------------------------------------------------

        for i in range(min(self.num_joints, len(q_deg))):

            self.target_deg[i] = q_deg[i]

            self.sliders[i].set(q_deg[i])

            self.slider_labels[i].config(
                text=f"{q_deg[i]:.1f}°"
            )

            self.entries[i].delete(0, tk.END)
            self.entries[i].insert(0, f"{q_deg[i]:.1f}")

        self.update_kinematics()

        if self.last_ik_des_target is not None:
            x_obj, y_obj, z_obj = self.last_ik_des_target
            self.update_mth_verification(
                self.ikd_mth,
                result.position[0], result.position[1], result.position[2],
                x_obj, y_obj, z_obj
            )
            self.add_trail_point(x_obj, y_obj, z_obj)

        if self._on_trajectory_ik_result('des'):
            return

        self.set_ik_des_status(
            "✓ Movimiento realizado.",
            "green"
        )


    # ========================================================
    # CINEMÁTICA INVERSA (DESACOPLE) - MENSAJE DE ESTADO
    # ========================================================

    def set_ik_des_status(self, message, color):

        self.ikd_status_label.config(
            text=message,
            foreground=color
        )


    # ========================================================
    # VERIFICACIÓN MTH - CONSTRUCCIÓN DEL WIDGET
    #
    # Se llama una vez por cada pestaña de cinemática inversa
    # (algebraica, Jacobiano, desacople). Devuelve un diccionario
    # con las referencias gráficas que después actualizan
    # update_mth_verification() / reset_mth_verification().
    # ========================================================

    def build_mth_verification(self, parent):

        frame = ttk.LabelFrame(
            parent,
            text="Verificación MTH (matriz de transformación homogénea)",
            padding=8,
            style='Section.TLabelframe'
        )

        frame.pack(
            fill='x',
            pady=(0, 8)
        )

        matrix_frame = ttk.Frame(frame)

        matrix_frame.pack(
            pady=(0, 5)
        )

        matrix_labels = []

        for row in range(4):

            matrix_row = []

            for col in range(4):

                label = tk.Label(
                    matrix_frame,
                    text="--",
                    width=10,
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

                matrix_row.append(label)

            matrix_labels.append(matrix_row)

        status_label = tk.Label(
            frame,
            text="Sin verificar todavía.",
            font=('Arial', 9, 'bold'),
            anchor='center'
        )

        status_label.pack(
            pady=(2, 0)
        )

        return {
            'matrix_labels': matrix_labels,
            'status_label': status_label,
        }


    # ========================================================
    # VERIFICACIÓN MTH - ACTUALIZAR CON UN RESULTADO
    #
    # Arma Tdes a partir del (q1,q2,q3) que calculó el método de
    # la pestaña (algebraico / Jacobiano / desacople), le pide al
    # método MTH que recupere esos mismos ángulos leyendo la
    # matriz, y muestra la comparación.
    # ========================================================

    def update_mth_verification(
        self, mth_widgets, q1, q2, q3, x_obj, y_obj, z_obj
    ):

        resultado = verificar_con_mth(q1, q2, q3, x_obj, y_obj, z_obj)

        T = resultado['T']

        for row in range(4):
            for col in range(4):
                mth_widgets['matrix_labels'][row][col].config(
                    text=f"{T[row, col]:.4f}"
                )

        if not resultado['ok']:

            mth_widgets['status_label'].config(
                text="✗ La verificación MTH no coincide (revisar).",
                foreground='red'
            )

            return

        mth_widgets['status_label'].config(
            text=(
                f"✓ MTH verificado — error posición: "
                f"{resultado['pos_error']:.4f} mm · "
                f"error ángulos: "
                f"{math.degrees(resultado['q_error']):.5f}°"
            ),
            foreground='dark green'
        )


    # ========================================================
    # VERIFICACIÓN MTH - LIMPIAR (cuando no hubo solución)
    # ========================================================

    def reset_mth_verification(self, mth_widgets):

        for row in mth_widgets['matrix_labels']:
            for label in row:
                label.config(text="--")

        mth_widgets['status_label'].config(
            text="Sin verificar (no alcanzable).",
            foreground='gray'
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
        # T05 -> índice 4
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
        # Si llegó un resultado nuevo de cinemática inversa,
        # lo aplicamos acá (hilo de Tkinter), nunca dentro de
        # on_ik_result (hilo de ROS).
        # ----------------------------------------------------

        if self.pending_ik_result is not None:

            self.apply_ik_result(
                self.pending_ik_result
            )

            self.pending_ik_result = None

        if self.pending_ik_jacob_result is not None:

            self.apply_ik_jacob_result(
                self.pending_ik_jacob_result
            )

            self.pending_ik_jacob_result = None

        if self.pending_ik_des_result is not None:

            self.apply_ik_des_result(
                self.pending_ik_des_result
            )

            self.pending_ik_des_result = None


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
