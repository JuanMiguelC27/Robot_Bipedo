"""
Cinemática inversa de la pata del robot bípedo.

Implementación por desacople (premultiplicación por matrices inversas).
Modelo nuevo: con articulación fantasma (1A2) y eslabón L6.
Pata IZQUIERDA (misma tabla y convención que forward_kinematics_left en
cinematica_directa_der_izq.py: alpha=+90° en 1A2, d=-L3 en 2A3;
+q2 lleva la pierna adelante, +q3 la lleva atrás).
"""

import numpy as np

L1, L2, L3, L4, L5, L6 = 147.03, 105.1, 159.95, 94.9, 311.97, 338.31

q1_deg, q2_deg, q3_deg = 0, 0, 0   # ángulos de prueba, para armar Tdes

# Límites articulares [°]
q1_min, q1_max = -160, 90
q2_min, q2_max = -115, 115
q3_min, q3_max = -85, 65


def dh_matrix(theta, d, a, alpha):
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0,   sa,     ca,    d   ],
        [0,   0,      0,     1   ]])


def fk_T(q1, q2, q3, L1, L2, L3, L4, L5, L6):
    A01 = dh_matrix(0,  d=-L1, a=L2, alpha=-np.pi/2)
    A12 = dh_matrix(q1, d=0,   a=0,  alpha=np.pi/2)    # fantasma
    A23 = dh_matrix(0,  d=-L3, a=L4, alpha=0)
    A34 = dh_matrix(q2, d=0,   a=L5, alpha=np.pi)
    A45 = dh_matrix(q3, d=0,   a=L6, alpha=0)
    return A01 @ A12 @ A23 @ A34 @ A45                 # = 0A5


def cinematica_inversa_pata_des(T, L1, L2, L3, L4, L5, L6, codo="arriba"):
    """
    Calcula q1, q2, q3 [rad] que producen la matriz T, despejando por
    desacople (premultiplicando por las matrices DH inversas en vez
    de leer celdas directamente, como hace el método MTH).

    Retorna (q1, q2, q3, alcanzable).
    """
    px, py, pz = T[0, 3], T[1, 3], T[2, 3]

    # q1: ecuación (pz+L1)*c1 + (px-L2)*s1 = -L3
    A = pz + L1
    B = px - L2
    C = -L3
    disc1 = A**2 + B**2 - C**2
    if disc1 < -1e-9:
        return None, None, None, False
    disc1 = max(disc1, 0.0)
    q1 = np.arctan2(B, A) - np.arctan2(np.sqrt(disc1), C)
    q1 = np.arctan2(np.sin(q1), np.cos(q1))    # <- envolver a (-180°, 180°]
    c1, s1 = np.cos(q1), np.sin(q1)

    # f14, f24: columna de posición de F = (1A2)^-1 (0A1)^-1 Tdes
    f14 = c1*(px - L2) - s1*(pz + L1)
    f24 = py

    # q2: A cos(q2) + B sin(q2) = C  (triángulo auxiliar con L5, L6)
    Aq2 = f14 - L4
    Bq2 = f24
    Cq2 = (Aq2**2 + Bq2**2 + L5**2 - L6**2) / (2*L5)

    disc2 = Aq2**2 + Bq2**2 - Cq2**2
    if disc2 < -1e-9:
        return None, None, None, False
    disc2 = max(disc2, 0.0)

    signo = 1 if codo == "arriba" else -1
    q2 = np.arctan2(Bq2, Aq2) + signo*np.arctan2(np.sqrt(disc2), Cq2)
    q2 = np.arctan2(np.sin(q2), np.cos(q2))    # <- envolver a (-180°, 180°]
    c2, s2 = np.cos(q2), np.sin(q2)

    # q3: de las dos ecuaciones de posición restantes
    q3 = np.arctan2(s2*Aq2 - c2*Bq2, c2*Aq2 + s2*Bq2 - L5)

    alcanzable = np.max(np.abs(fk_T(q1, q2, q3, L1, L2, L3, L4, L5, L6) - T)) < 1e-6
    return q1, q2, q3, alcanzable

def joint_limit_warnings(q1_deg, q2_deg, q3_deg, tol=1e-6):
    warnings = []
    for nombre, v, mn, mx in (("q1", q1_deg, q1_min, q1_max),
                              ("q2", q2_deg, q2_min, q2_max),
                              ("q3", q3_deg, q3_min, q3_max)):
        if not (mn - tol) <= v <= (mx + tol):
            warnings.append(f"{nombre} fuera de rango [{mn}°, {mx}°]")
    return warnings


def cinematica_inversa_pata_des_xyz(x, y, z, L1=L1, L2=L2, L3=L3, L4=L4,
                                    L5=L5, L6=L6, codo="arriba"):
    """
    Versión para un punto (x, y, z) [mm], que es lo que mandan los
    nodos ROS y la interfaz. El desacople solo usa la posición de T
    para despejar q1, q2, q3; la orientación no se puede elegir
    (3 GDL), así que el alcance se verifica solo por posición.

    Retorna (q1, q2, q3, alcanzable).
    """
    T = np.eye(4)
    T[:3, 3] = (x, y, z)

    q1, q2, q3, _ = cinematica_inversa_pata_des(
        T, L1, L2, L3, L4, L5, L6, codo=codo)

    if q1 is None:
        return None, None, None, False

    p = fk_T(q1, q2, q3, L1, L2, L3, L4, L5, L6)[:3, 3]
    alcanzable = bool(np.max(np.abs(p - T[:3, 3])) < 1e-6)
    return q1, q2, q3, alcanzable


if __name__ == "__main__":
    qd = np.radians([q1_deg, q2_deg, q3_deg])
    Tdes = fk_T(*qd, L1, L2, L3, L4, L5, L6)

    q1, q2, q3, alcanzable = cinematica_inversa_pata_des(Tdes, L1, L2, L3, L4, L5, L6, codo="arriba")

    print(f"{'='*50}")
    print(f"q deseados: q1 = {q1_deg}°  q2 = {q2_deg}°  q3 = {q3_deg}°")
    print(f"Tdes:\n{np.round(Tdes, 4)}")
    print(f"{'='*50}")

    if not alcanzable:
        print("No alcanzable")
    else:
        q1d, q2d, q3d = np.degrees([q1, q2, q3])
        print(f"q1 = {q1d:.2f}°  q2 = {q2d:.2f}°  q3 = {q3d:.2f}°")
        print("error T:", np.max(np.abs(fk_T(q1, q2, q3, L1, L2, L3, L4, L5, L6) - Tdes)))
        for aviso in joint_limit_warnings(q1d, q2d, q3d):
            print(f"Advertencia: {aviso}")
