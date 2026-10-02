"""
Cinemática inversa - pierna de 3 GDL (método del jacobiano)
Implementación manual con Denavit-Hartenberg.

Q1 = cadera-roll, Q2 = cadera-pitch, Q3 = rodilla-pitch
Marco {0}: origen en el centro de la pelvis, X vertical hacia ABAJO,
Z lateral hacia la cadera, Y = Z x X (anteroposterior).

A diferencia de kinem_invers_leg_algebraico.py (solución cerrada),
este método es ITERATIVO: necesita una postura inicial (semilla)
q1, q2, q3 desde donde arrancar a converger hacia el objetivo.
"""

import numpy as np

# ----------------------------------------------------------------------
# Parámetros geométricos [mm] (mismos que kinem_leg_gen.py)
# ----------------------------------------------------------------------
L1 = 200.4
L2 = 83.75
L3 = 118.78
L4 = 253.2
L5 = 253.29

# Límites articulares [°] (sólo se avisa si la solución se sale)
q1_min, q1_max = -160, 70
q2_min, q2_max = -115, 115
q3_min, q3_max = -85, 65

# Parámetros del método
ALFA = 0.5         # factor de paso (0 < alfa <= 1)
TOL = 0.001         # error de posición aceptado [mm]
MAX_ITER = 200      # iteraciones máximas
PASO_MAX = 15       # giro máximo por iteración [°]
LAM = 1.0           # amortiguamiento λ [mm], sólo cerca de singularidades
RAMA = +1           # +1: rodilla con Q3 > 0   |   -1: rodilla con Q3 < 0


# ----------------------------------------------------------------------
# Matriz DH individual
# ----------------------------------------------------------------------
def dh_matrix(theta, d, a, alpha):
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0,   sa,     ca,    d   ],
        [0,   0,      0,     1   ]
    ])


# ----------------------------------------------------------------------
# Cinemática directa: p = última columna de 0A4
# ----------------------------------------------------------------------
def cinematica_directa(q1, q2, q3, L1=L1, L2=L2, L3=L3, L4=L4, L5=L5):
    A01 = dh_matrix(0,  d=L1, a=L2, alpha=np.pi/2)
    A12 = dh_matrix(q1, d=0,  a=L3, alpha=-np.pi/2)
    A23 = dh_matrix(q2, d=0,  a=L4, alpha=np.pi)
    A34 = dh_matrix(q3, d=0,  a=L5, alpha=0)
    A04 = A01 @ A12 @ A23 @ A34
    return A04[:3, 3]                      # [X, Y, Z]


# ----------------------------------------------------------------------
# Jacobiano: derivadas parciales de X, Y, Z respecto a Q1, Q2, Q3
#   X = L2 + r cos Q1      r = L3 + L4 cos Q2 + L5 cos(Q2 - Q3)
#   Y = w                  w = L4 sin Q2 + L5 sin(Q2 - Q3)
#   Z = L1 + r sin Q1      det J = L4 L5 r sin Q3
# ----------------------------------------------------------------------
def jacobiano(q1, q2, q3, L3=L3, L4=L4, L5=L5):
    phi = q2 - q3
    r = L3 + L4*np.cos(q2) + L5*np.cos(phi)
    w = L4*np.sin(q2) + L5*np.sin(phi)
    return np.array([
        #  ∂/∂Q1          ∂/∂Q2                           ∂/∂Q3
        [-r*np.sin(q1),  -w*np.cos(q1),                  L5*np.sin(phi)*np.cos(q1)],  # X
        [ 0,              L4*np.cos(q2) + L5*np.cos(phi), -L5*np.cos(phi)            ],  # Y
        [ r*np.cos(q1),  -w*np.sin(q1),                  L5*np.sin(phi)*np.sin(q1)],  # Z
    ])


# ----------------------------------------------------------------------
# Método iterativo:  e = p_obj - p(Q),  ΔQ = J^-1·e,  Q <- Q + alfa·ΔQ
# ----------------------------------------------------------------------
def cinematica_inversa_pata_jacob(x, y, z,
                                   q1_seed_deg, q2_seed_deg, q3_seed_deg,
                                   L1=L1, L2=L2, L3=L3, L4=L4, L5=L5,
                                   alfa=ALFA, tol=TOL, max_iter=MAX_ITER,
                                   paso_max=PASO_MAX, lam=LAM, rama=RAMA):
    """
    Calcula q1, q2, q3 [rad] que llevan el pie a (x, y, z) [mm],
    partiendo de la semilla (q1_seed_deg, q2_seed_deg, q3_seed_deg) [°].

    Retorna (q1, q2, q3, alcanzable). alcanzable es False si el
    método no converge dentro de max_iter iteraciones.
    """
    p_obj = np.array([x, y, z], dtype=float)
    q = np.radians([q1_seed_deg, q2_seed_deg, q3_seed_deg]).astype(float)
    eps = np.radians(0.01)
    alcanzable = False

    for _ in range(max_iter + 1):
        q[2] = rama * np.clip(rama*q[2], eps, np.pi - eps)   # misma rama de rodilla
        e = p_obj - cinematica_directa(*q, L1=L1, L2=L2, L3=L3, L4=L4, L5=L5)

        if np.linalg.norm(e) <= tol:
            alcanzable = True
            break

        J = jacobiano(*q, L3=L3, L4=L4, L5=L5)

        if abs(np.linalg.det(J)) / (L1+L2+L3+L4+L5)**3 > 1e-4:
            dq = np.linalg.solve(J, e)                                  # ΔQ = J^-1·e
        else:
            dq = J.T @ np.linalg.solve(J @ J.T + lam**2*np.eye(3), e)   # singular: amortiguado

        dq *= alfa
        mayor = np.max(np.abs(dq))
        if mayor > np.radians(paso_max):                      # limita el giro por iteración
            dq *= np.radians(paso_max) / mayor
        q += dq

    if not alcanzable:
        return None, None, None, False

    return q[0], q[1], q[2], True


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
