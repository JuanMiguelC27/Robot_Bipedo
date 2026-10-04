"""
Cinemática inversa de la pata del robot bípedo.

Implementación por matriz final MTH (roll-pitch-pitch con offset DH).
Mismos parámetros geométricos y convención DH que
kinem_invers_leg_jacob.py y kinem_invers_leg_mth_Desacople.py.

Este método no recibe un punto (x, y, z): recibe la matriz de
transformación homogénea T deseada (posición + orientación), porque
despeja los ángulos leyendo celdas puntuales de esa matriz. Como la
pata tiene 3 GDL, la orientación no es libre: es consecuencia de
q1, q2, q3, igual que la posición. Por eso se usa como VERIFICACIÓN
de los otros métodos (algebraico, Jacobiano, desacople): se arma T
a partir del q que esos métodos ya calcularon, y se comprueba que el
método matricial recupera esos mismos ángulos (ver verificar_con_mth).
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


def dh_matrix(theta, d, a, alpha):
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0,   sa,     ca,    d   ],
        [0,   0,      0,     1   ]])


def params_dh(q1, q2, q3, L1=L1, L2=L2, L3=L3, L4=L4, L5=L5):
    return [(0,  L1, L2,  np.pi/2),
            (q1, 0,  L3, -np.pi/2),
            (q2, 0,  L4,  np.pi),
            (q3, 0,  L5,  0)]


def fk_T(q1, q2, q3, L1=L1, L2=L2, L3=L3, L4=L4, L5=L5):
    T = np.eye(4)
    for th, d, a, al in params_dh(q1, q2, q3, L1, L2, L3, L4, L5):
        T = T @ dh_matrix(th, d, a, al)
    return T                                   # = 0A4


def cinematica_inversa_pata_mth(T, L1=L1, L2=L2, L3=L3, L4=L4, L5=L5):
    """
    Calcula los ángulos articulares q1, q2, q3 [rad] que producen la
    matriz de transformación homogénea T, leyendo directamente sus
    celdas (en vez de resolver un triángulo geométrico).

    Retorna (q1, q2, q3, alcanzable). alcanzable es False si T no es
    realizable con los 3 GDL de la pata (fk_T(q) no reproduce T).
    """
    nx, ny = T[0, 0], T[1, 0]
    oy = T[1, 1]
    ax, az = T[0, 2], T[2, 2]
    px, py, pz = T[0, 3], T[1, 3], T[2, 3]

    q1 = np.arctan2(ax, -az)
    th = np.arctan2(ny, -oy)                   # th = q2 - q3
    r  = (px - L2)*np.cos(q1) + (pz - L1)*np.sin(q1)
    s2 = (py - L5*ny) / L4
    c2 = (r - L3 + L5*oy) / L4
    q2 = np.arctan2(s2, c2)
    q3 = q2 - th
    q3 = np.arctan2(np.sin(q3), np.cos(q3))    # a (-180°, 180°]

    # Validar: la T debe ser realizable con 3 GDL
    alcanzable = np.max(np.abs(fk_T(q1, q2, q3, L1, L2, L3, L4, L5) - T)) < 1e-6
    return q1, q2, q3, alcanzable


def verificar_con_mth(q1, q2, q3, x_obj=None, y_obj=None, z_obj=None,
                       L1=L1, L2=L2, L3=L3, L4=L4, L5=L5, tol=1e-6):
    """
    Verifica, con el método de la matriz de transformación homogénea,
    la solución (q1, q2, q3) [rad] obtenida por otro método de
    cinemática inversa (algebraico, Jacobiano o desacople).

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
        pos_error -> error de posición [mm] entre Tdes y (x_obj,y_obj,z_obj),
                     o None si no se dio el objetivo
        ok        -> True si fue alcanzable y q_error < tol
    """
    Tdes = fk_T(q1, q2, q3, L1, L2, L3, L4, L5)
    q1r, q2r, q3r, alcanzable = cinematica_inversa_pata_mth(
        Tdes, L1, L2, L3, L4, L5)

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
    # Ángulos deseados [°], de prueba
    q1_deg, q2_deg, q3_deg = 20, 40, 30

    qd = np.radians([q1_deg, q2_deg, q3_deg])
    Tdes = fk_T(*qd)

    q1, q2, q3, alcanzable = cinematica_inversa_pata_mth(Tdes)

    print(f"{'='*50}")
    print(f"q deseados: q1 = {q1_deg}°  q2 = {q2_deg}°  q3 = {q3_deg}°")
    print(f"Tdes:\n{np.round(Tdes, 4)}")
    print(f"{'='*50}")

    if not alcanzable:
        print("No alcanzable")
    else:
        q1d, q2d, q3d = np.degrees([q1, q2, q3])
        print(f"q1 = {q1d:.2f}°  q2 = {q2d:.2f}°  q3 = {q3d:.2f}°")
        print("error T:", np.max(np.abs(fk_T(q1, q2, q3) - Tdes)))
