"""
Cinemática inversa de la pata del robot bípedo.

Implementación por desacople (premultiplicación por matrices inversas).
Mismos parámetros geométricos y convención DH que kinem_invers_leg_jacob.py
y kinem_invers_leg_mth.py.
"""

import numpy as np

L1 = 200.4
L2 = 83.75
L3 = 118.78
L4 = 253.2
L5 = 253.29

# Límites articulares [°]
q1_min, q1_max = -160, 70
q2_min, q2_max = -115, 115
q3_min, q3_max = -85, 65


def cinematica_inversa_pata_des(x, y, z,
                                 L1=L1, L2=L2, L3=L3, L4=L4, L5=L5,
                                 codo="arriba"):
    """
    Calcula los ángulos articulares q1, q2, q3 [rad] necesarios para
    que el extremo de la pata alcance el punto (x, y, z) [mm],
    despejando por desacople (premultiplicando por las matrices DH
    inversas en vez de resolver el triángulo por ley del coseno).

    Retorna (q1, q2, q3, alcanzable). Si el punto no es alcanzable
    geométricamente, retorna (None, None, None, False).
    """
    # q1: fila 3, columna 4  ->  -S1(Px-L2) + C1(Pz-L1) = 0
    xp = x - L2
    zp = z - L1
    q1 = np.arctan2(zp, xp)

    # f14: fila 1, columna 4 de (1A2)^-1 (0A1)^-1 Tdes
    f14 = np.cos(q1)*xp + np.sin(q1)*zp - L3

    # q2: A cos(q2) + B sin(q2) = C  (triángulo auxiliar)
    A = f14
    B = y
    C = (L5**2 - f14**2 - y**2 - L4**2) / (-2*L4)

    # Validar alcance: A^2 + B^2 - C^2 >= 0
    d = A**2 + B**2 - C**2
    if d < -1e-9:                          # tolerancia numérica en el borde
        return None, None, None, False
    d = max(d, 0.0)

    signo = -1 if codo == "arriba" else 1  # arriba: q3 > 0 | abajo: q3 < 0
    q2 = np.arctan2(B, A) - np.arctan2(signo*np.sqrt(d), C)

    # q3: filas 1 y 2, columna 4
    c3 = (np.cos(q2)*f14 + np.sin(q2)*y - L4) / L5
    s3 = (np.sin(q2)*f14 - np.cos(q2)*y) / L5
    q3 = np.arctan2(s3, c3)

    return q1, q2, q3, True


def joint_limit_warnings(q1_deg, q2_deg, q3_deg):
    """
    Devuelve una lista de textos de advertencia para los ángulos
    (en grados) que queden fuera de los límites articulares
    definidos arriba. Lista vacía si todos están dentro de rango.
    """
    warnings = []
    for nombre, v, mn, mx in (("q1", q1_deg, q1_min, q1_max),
                              ("q2", q2_deg, q2_min, q2_max),
                              ("q3", q3_deg, q3_min, q3_max)):
        if not mn <= v <= mx:
            warnings.append(f"{nombre} fuera de rango [{mn}°, {mx}°]")
    return warnings


if __name__ == "__main__":
    x, y, z = 4, 0, 1

    q1, q2, q3, alcanzable = cinematica_inversa_pata_des(
        x, y, z, L1=1, L2=1, L3=1, L4=1, L5=1, codo="arriba")

    print(f"{'='*50}")
    print(f"Objetivo: x = {x}  y = {y}  z = {z}")
    print(f"{'='*50}")

    if not alcanzable:
        print("No alcanzable")
    else:
        q1d, q2d, q3d = np.degrees([q1, q2, q3])
        print(f"q1 = {q1d:.2f}°  q2 = {q2d:.2f}°  q3 = {q3d:.2f}°")
