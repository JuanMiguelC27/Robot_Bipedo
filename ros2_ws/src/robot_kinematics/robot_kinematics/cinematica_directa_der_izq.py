"""
Cinemática directa del robot bípedo.

Implementación manual mediante Denavit-Hartenberg.
Los ángulos de entrada se manejan en grados.

Tabla DH (con articulación fantasma 1A2 para reorientar
el eje de cadera roll hacia el plano de la rodilla):

  Pierna DERECHA:
        theta   d    a     alpha
  0A1    0     L1    L2     90°
  1A2    q1    0     0     -90°   <- fantasma (reorienta eje)
  2A3    0     L3    L4     0
  3A4    q2    0     L5     180°
  4A5    q3    0     L6     0

  Pierna IZQUIERDA (la misma tabla de las cinemáticas inversas *_izq):
        theta   d    a     alpha
  0A1    0    -L1    L2    -90°
  1A2    q1    0     0      90°   <- fantasma (reorienta eje)
  2A3    0    -L3    L4     0
  3A4    q2    0     L5     180°
  4A5    q3    0     L6     0

  Convención de ángulos (la misma del URDF V9, cuyo Base_link
  coincide con la base {0}: +X abajo, +Y al FRENTE, +Z lateral):
      +q2 (cadera pitch) -> la pierna va hacia ADELANTE (+Y)
      +q3 (rodilla)      -> la pierna va hacia ATRÁS   (-Y)
  Por eso en la izquierda 1A2 lleva alpha=+90° y 2A3 d=-L3: con
  alpha=-90° y d=+L3 la posición sale reflejada en Y y los
  sentidos de q2 y q3 quedan al revés.
"""

import numpy as np


# ----------------------------------------------------------------------
# Parámetros geométricos
# ----------------------------------------------------------------------
#L1, L2, L3, L4, L5, L6 = 14.703, 10.51, 15.995, 9.49, 31.197, 33.831

L1 = 147.03
L2 = 105.1
L3 = 159.95
L4 = 94.9
L5 = 311.97
L6 = 338.31 


# ----------------------------------------------------------------------
# Matriz de transformación DH
# ----------------------------------------------------------------------

def dh_matrix(theta, d, a, alpha):
    """
    Calcula la matriz homogénea 4x4
    utilizando la convención DH estándar.

    Parámetros:
        theta : ángulo articular [rad]
        d     : desplazamiento [m]
        a     : longitud del eslabón [m]
        alpha : ángulo de torsión [rad]
    """

    ct = np.cos(theta)
    st = np.sin(theta)

    ca = np.cos(alpha)
    sa = np.sin(alpha)

    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0,        sa,       ca,      d],
        [0,         0,        0,      1]
    ])


# ----------------------------------------------------------------------
# CINEMÁTICA PIERNA DERECHA
# ----------------------------------------------------------------------

def forward_kinematics_right(q):
    """
    Calcula las transformaciones homogéneas de la pierna derecha.

    q:
        [hip_roll, hip_pitch, knee] en grados

    Retorna:
        T01, T02, T03, T04, T05
    """

    if len(q) != 3:
        raise ValueError(
            "La pierna derecha debe recibir exactamente 3 ángulos."
        )

    q1, q2, q3 = np.radians(q)

    # --------------------------------------------------------------
    # Transformaciones DH
    # --------------------------------------------------------------

    A01 = dh_matrix(
        0,
        d=L1,
        a=L2,
        alpha=np.pi / 2
    )

    A12 = dh_matrix(          # articulación fantasma (hip_roll)
        q1,
        d=0,
        a=0,
        alpha=-np.pi / 2
    )

    A23 = dh_matrix(
        0,
        d=L3,
        a=L4,
        alpha=0
    )

    A34 = dh_matrix(          # hip_pitch
        q2,
        d=0,
        a=L5,
        alpha=np.pi
    )

    A45 = dh_matrix(          # knee
        q3,
        d=0,
        a=L6,
        alpha=0
    )

    # --------------------------------------------------------------
    # Transformaciones acumuladas
    # --------------------------------------------------------------

    T01 = A01
    T02 = T01 @ A12
    T03 = T02 @ A23
    T04 = T03 @ A34
    T05 = T04 @ A45

    return T01, T02, T03, T04, T05


# ----------------------------------------------------------------------
# CINEMÁTICA PIERNA IZQUIERDA
# ----------------------------------------------------------------------

def forward_kinematics_left(q):
    """
    Calcula las transformaciones homogéneas de la pierna izquierda.

    q:
        [hip_roll, hip_pitch, knee] en grados

    Retorna:
        T01, T02, T03, T04, T05
    """

    if len(q) != 3:
        raise ValueError(
            "La pierna izquierda debe recibir exactamente 3 ángulos."
        )

    q1, q2, q3 = np.radians(q)

    # --------------------------------------------------------------
    # Transformaciones DH (se refleja el signo de L1, L3 y de los
    # alpha de 0A1 y 1A2 respecto a la pierna derecha; ver la
    # convención de ángulos al inicio del archivo)
    # --------------------------------------------------------------

    A01 = dh_matrix(
        0,
        d=-L1,
        a=L2,
        alpha=-np.pi / 2
    )

    A12 = dh_matrix(          # articulación fantasma (hip_roll)
        q1,
        d=0,
        a=0,
        alpha=np.pi / 2
    )

    A23 = dh_matrix(
        0,
        d=-L3,
        a=L4,
        alpha=0
    )

    A34 = dh_matrix(          # hip_pitch
        q2,
        d=0,
        a=L5,
        alpha=np.pi
    )

    A45 = dh_matrix(          # knee
        q3,
        d=0,
        a=L6,
        alpha=0
    )

    # --------------------------------------------------------------
    # Transformaciones acumuladas
    # --------------------------------------------------------------

    T01 = A01
    T02 = T01 @ A12
    T03 = T02 @ A23
    T04 = T03 @ A34
    T05 = T04 @ A45

    return T01, T02, T03, T04, T05


# ----------------------------------------------------------------------
# CINEMÁTICA COMPLETA DEL BÍPEDO
# ----------------------------------------------------------------------

def forward_kinematics_biped(q_right, q_left):
    """
    Calcula todas las transformaciones de ambas piernas.

    Retorna:

        {
            "right": (T01, T02, T03, T04, T05),
            "left":  (T01, T02, T03, T04, T05)
        }
    """

    right = forward_kinematics_right(q_right)
    left = forward_kinematics_left(q_left)

    return {
        "right": right,
        "left": left
    }


# ----------------------------------------------------------------------
# Posición
# ----------------------------------------------------------------------

def get_position(T):
    """
    Extrae la posición XYZ de una MTH.
    """

    return T[0:3, 3]


# ----------------------------------------------------------------------
# Rotación
# ----------------------------------------------------------------------

def get_rotation(T):
    """
    Extrae la matriz de rotación 3x3.
    """

    return T[0:3, 0:3]


# ----------------------------------------------------------------------
# Formato para mostrar en la interfaz
# ----------------------------------------------------------------------

def format_matrix(T):
    """
    Convierte una matriz 4x4 en texto para mostrarla en Tkinter.
    """

    return (
        f"{T[0,0]:8.4f}  {T[0,1]:8.4f}  {T[0,2]:8.4f}  {T[0,3]:8.4f}\n"
        f"{T[1,0]:8.4f}  {T[1,1]:8.4f}  {T[1,2]:8.4f}  {T[1,3]:8.4f}\n"
        f"{T[2,0]:8.4f}  {T[2,1]:8.4f}  {T[2,2]:8.4f}  {T[2,3]:8.4f}\n"
        f"{T[3,0]:8.4f}  {T[3,1]:8.4f}  {T[3,2]:8.4f}  {T[3,3]:8.4f}"
    )