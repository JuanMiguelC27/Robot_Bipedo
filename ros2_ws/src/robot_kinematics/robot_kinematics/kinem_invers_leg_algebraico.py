"""
Cinemática inversa de la pata del robot bípedo.

Implementación algebraica (roll-pitch-pitch con offset DH).
Mismos parámetros geométricos que kinem_leg_gen.py.
"""

import numpy as np

L1 = 200.4
L2 = 83.75
L3 = 118.78
L4 = 253.2
L5 = 253.29

# Límites articulares [°]
q1_min, q1_max = -165, 90
q2_min, q2_max = 45, 225
q3_min, q3_max = 45, 225


def cinematica_inversa_pata_alg(x, y, z,
                                 L1=L1, L2=L2, L3=L3, L4=L4, L5=L5,
                                 codo="arriba"):
    """
    Calcula los ángulos articulares q1, q2, q3 [rad] necesarios para
    que el extremo de la pata alcance el punto (x, y, z).

    Retorna (q1, q2, q3, alcanzable). Si el punto no es alcanzable
    geométricamente, retorna (None, None, None, False).
    """
    xp = x - L2
    zp = z - L1
    q1 = np.arctan2(zp, xp)
    rho = np.sqrt(xp**2 + zp**2)

    c3 = ((rho - L3)**2 + y**2 - L4**2 - L5**2) / (2*L4*L5)
    if abs(c3) > 1 + 1e-9:                 # tolerancia numérica en el borde
        return None, None, None, False
    c3 = np.clip(c3, -1.0, 1.0)

    s3 = np.sqrt(1 - c3**2) * (1 if codo == "arriba" else -1)
    q3 = np.arctan2(s3, c3)

    A = L4 + L5*np.cos(q3)
    B = L5*np.sin(q3)
    det = A**2 + B**2
    c2 = (A*(rho - L3) - B*y) / det
    s2 = (B*(rho - L3) + A*y) / det
    q2 = np.arctan2(s2, c2)

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
