"""
Cinemática directa de una pierna del robot bípedo.

Implementación manual mediante Denavit-Hartenberg.

Convención:
    q[0] -> Hip Roll
    q[1] -> Hip Pitch
    q[2] -> Knee Pitch

Las funciones de cinemática reciben los ángulos en grados
y retornan las matrices homogéneas T01, T02, T03 y T04.
"""

import numpy as np


# ======================================================================
# PARÁMETROS GEOMÉTRICOS
# ======================================================================

L1 = 200.0
L2 = 85.0
L3 = 120.0
L4 = 235.0
L5 = 250.0

# ======================================================================
# MATRIZ DH
# ======================================================================

def dh_matrix(theta, d, a, alpha):
    """
    Calcula una matriz homogénea 4x4 utilizando
    la convención DH estándar.
    """

    ct = np.cos(theta)
    st = np.sin(theta)

    ca = np.cos(alpha)
    sa = np.sin(alpha)

    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0, sa,       ca,      d],
        [0.0, 0.0,      0.0,     1.0]
    ])


# ======================================================================
# CINEMÁTICA DIRECTA - PIERNA DERECHA
# ======================================================================

def forward_kinematics_right(q):
    """
    Calcula la cinemática directa de la pierna derecha.

    Parámetros
    ----------
    q : list o array
        [Hip Roll, Hip Pitch, Knee Pitch] en grados.

    Retorna
    -------
    T01, T02, T03, T04 : np.ndarray
        Matrices homogéneas 4x4.
    """

    # --------------------------------------------------------------
    # Conversión a radianes
    # --------------------------------------------------------------

    q1 = np.radians(q[0])
    q2 = np.radians(q[1])
    q3 = np.radians(q[2])

    # --------------------------------------------------------------
    # Matrices DH
    # --------------------------------------------------------------

    # Articulación 1 - Hip Roll
    A01 = dh_matrix(
        0,
        d=L1,
        a=L2,
        alpha=np.pi / 2
    )

    # Articulación 2 - Hip Pitch
    A12 = dh_matrix(
        q1,
        d=0,
        a=L3,
        alpha=-np.pi / 2
    )

    # Articulación 3 - Knee Pitch
    A23 = dh_matrix(
        q2,
        d=0,
        a=L4,
        alpha=np.pi
    )

    # Articulación 4
    A34 = dh_matrix(
        q3,
        d=0,
        a=L5,
        alpha=0
    )

    # --------------------------------------------------------------
    # Transformaciones acumuladas
    # --------------------------------------------------------------

    T01 = A01

    T02 = A01 @ A12

    T03 = T02 @ A23

    T04 = T03 @ A34

    return T01, T02, T03, T04


# ======================================================================
# CINEMÁTICA DIRECTA - PIERNA IZQUIERDA
# ======================================================================

def forward_kinematics_left(q):
    """
    Calcula la cinemática directa de la pierna izquierda.

    IMPORTANTE:
    Por ahora utiliza la misma cadena DH de la versión original.
    Si la pierna izquierda tiene una convención DH reflejada,
    esta función debe modificarse con esa tabla DH específica.
    """

    # --------------------------------------------------------------
    # Conversión a radianes
    # --------------------------------------------------------------

    q1 = np.radians(q[0])
    q2 = np.radians(q[1])
    q3 = np.radians(q[2])

    # --------------------------------------------------------------
    # Matrices DH
    # --------------------------------------------------------------

    # Articulación 1 - Hip Roll
    A01 = dh_matrix(
        0,
        d=-L1,
        a=L2,
        alpha=-np.pi / 2
    )

    # Articulación 2 - Hip Pitch
    A12 = dh_matrix(
        q1,
        d=0,
        a=L3,
        alpha=np.pi / 2
    )

    # Articulación 3 - Knee Pitch
    A23 = dh_matrix(
        q2,
        d=0,
        a=L4,
        alpha=np.pi
    )

    # Articulación 4
    A34 = dh_matrix(
        q3,
        d=0,
        a=L5,
        alpha=0
    )

    # --------------------------------------------------------------
    # Transformaciones acumuladas
    # --------------------------------------------------------------

    T01 = A01

    T02 = A01 @ A12

    T03 = T02 @ A23

    T04 = T03 @ A34

    return T01, T02, T03, T04


# ======================================================================
# OBTENER POSICIÓN XYZ
# ======================================================================

def get_position(T):
    """
    Extrae la posición [x, y, z] de una matriz homogénea.
    """

    return T[0:3, 3]


# ======================================================================
# MOSTRAR MATRIZ
# ======================================================================

def imprimir_matriz(nombre, M):
    """
    Imprime una matriz de forma legible.
    """

    print(f"\n{nombre} =")
    print(np.round(M, 4))


# ======================================================================
# PRUEBA DEL MÓDULO
# ======================================================================

if __name__ == "__main__":

    # Ángulos de prueba
    q = [0.0, 0.0, 0.0]

    T01, T02, T03, T04 = forward_kinematics_right(q)

    print("=" * 50)
    print(
        f"Hip Roll = {q[0]}° | "
        f"Hip Pitch = {q[1]}° | "
        f"Knee Pitch = {q[2]}°"
    )
    print("=" * 50)

    imprimir_matriz("T01", T01)
    imprimir_matriz("T02", T02)
    imprimir_matriz("T03", T03)
    imprimir_matriz("T04", T04)

    position = get_position(T04)

    print("\nPosición del efector final:")
    print(f"x = {position[0]:.4f}")
    print(f"y = {position[1]:.4f}")
    print(f"z = {position[2]:.4f}")
