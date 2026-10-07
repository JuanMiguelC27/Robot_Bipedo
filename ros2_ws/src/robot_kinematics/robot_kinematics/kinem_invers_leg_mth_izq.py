"""
Cinemática inversa de la pata del robot bípedo.

Implementación por matriz final MTH (roll-pitch-pitch con offset DH).
Modelo nuevo: con articulación fantasma (1A2) y eslabón L6.
Pata IZQUIERDA (misma tabla y convención que forward_kinematics_left en
cinematica_directa_der_izq.py: alpha=+90° en 1A2, d=-L3 en 2A3;
+q2 lleva la pierna adelante, +q3 la lleva atrás).

Este método no recibe un punto (x, y, z): recibe la matriz de
transformación homogénea T deseada (posición + orientación), porque
despeja los ángulos leyendo celdas puntuales de esa matriz. Como la
pata tiene 3 GDL, la orientación no es libre: es consecuencia de
q1, q2, q3, igual que la posición.
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
    A12 = dh_matrix(q1, d=0,   a=0,  alpha=np.pi/2)   # fantasma
    A23 = dh_matrix(0,  d=-L3, a=L4, alpha=0)
    A34 = dh_matrix(q2, d=0,   a=L5, alpha=np.pi)
    A45 = dh_matrix(q3, d=0,   a=L6, alpha=0)
    return A01 @ A12 @ A23 @ A34 @ A45                 # = 0A5


def cinematica_inversa_pata_mth(T, L1, L2, L3, L4, L5, L6):
    """
    Calcula q1, q2, q3 [rad] leyendo directamente las celdas de T,
    en vez de resolver un triángulo geométrico.

    Retorna (q1, q2, q3, alcanzable).
    """
    nx, ny, nz = T[0, 0], T[1, 0], T[2, 0]
    oy = T[1, 1]
    ax, az = T[0, 2], T[2, 2]
    px, py, pz = T[0, 3], T[1, 3], T[2, 3]

    # Con la tabla de fk_T:
    #   ax = -sin(q1)      az = -cos(q1)
    #   ny = sin(q2-q3)    oy = -cos(q2-q3)
    #   py = L5 sin(q2) + L6 sin(q2-q3)
    q1 = np.arctan2(-ax, -az)
    c1, s1 = np.cos(q1), np.sin(q1)

    s2 = (py - L6*ny) / L5
    c2 = (c1*(px - L2) - s1*(pz + L1) - L4 - L6*(c1*nx - s1*nz)) / L5
    q2 = np.arctan2(s2, c2)

    q23 = np.arctan2(ny, -oy)                  # q2 - q3
    q3 = q2 - q23
    q3 = np.arctan2(np.sin(q3), np.cos(q3))    # a (-180°, 180°]

    alcanzable = np.max(np.abs(fk_T(q1, q2, q3, L1, L2, L3, L4, L5, L6) - T)) < 1e-6
    return q1, q2, q3, alcanzable


def verificar_con_mth(q1, q2, q3, x_obj=None, y_obj=None, z_obj=None,
                       L1=L1, L2=L2, L3=L3, L4=L4, L5=L5, L6=L6, tol=1e-6):
    """
    Verifica, con el método de la matriz de transformación homogénea,
    la solución (q1, q2, q3) [rad] obtenida por otro método de
    cinemática inversa (algebraico, geométrico, Jacobiano o desacople).

    1. Arma Tdes = fk_T(q1, q2, q3)  (cinemática directa).
    2. Le pasa Tdes al método MTH, que intenta recuperar los mismos
       ángulos leyendo únicamente la matriz.
    3. Compara los ángulos recuperados contra los originales y,
       si se dan x_obj/y_obj/z_obj, la posición de Tdes contra el
       punto que se le había pedido al método original.

    Retorna un diccionario:
        T         -> matriz Tdes (4x4)
        q1r/q2r/q3r -> ángulos [rad] recuperados por el método MTH
        q_error   -> error angular máximo [rad] entre q original y recuperado
        pos_error -> error de posición entre Tdes y (x_obj,y_obj,z_obj),
                     o None si no se dio el objetivo
        ok        -> True si fue alcanzable y q_error < tol
    """
    Tdes = fk_T(q1, q2, q3, L1, L2, L3, L4, L5, L6)
    q1r, q2r, q3r, alcanzable = cinematica_inversa_pata_mth(
        Tdes, L1, L2, L3, L4, L5, L6)

    if not alcanzable:
        return {
            'T': Tdes,
            'q1r': None, 'q2r': None, 'q3r': None,
            'q_error': None, 'pos_error': None,
            'ok': False,
        }

    def dif_angular(a, b):
        return abs(np.arctan2(np.sin(a - b), np.cos(a - b)))

    q_error = max(
        dif_angular(q1, q1r),
        dif_angular(q2, q2r),
        dif_angular(q3, q3r),
    )

    pos_error = None
    if x_obj is not None:
        pos_error = float(np.linalg.norm(
            Tdes[:3, 3] - np.array([x_obj, y_obj, z_obj])
        ))

    return {
        'T': Tdes,
        'q1r': q1r, 'q2r': q2r, 'q3r': q3r,
        'q_error': float(q_error),
        'pos_error': pos_error,
        'ok': bool(q_error < tol),
    }


if __name__ == "__main__":
    qd = np.radians([q1_deg, q2_deg, q3_deg])
    Tdes = fk_T(*qd, L1, L2, L3, L4, L5, L6)

    q1, q2, q3, alcanzable = cinematica_inversa_pata_mth(Tdes, L1, L2, L3, L4, L5, L6)

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
        for nombre, v, mn, mx in (("q1", q1d, q1_min, q1_max),
                                  ("q2", q2d, q2_min, q2_max),
                                  ("q3", q3d, q3_min, q3_max)):
            if not mn <= v <= mx:
                print(f"Advertencia: {nombre} fuera de rango [{mn}°, {mx}°]")
