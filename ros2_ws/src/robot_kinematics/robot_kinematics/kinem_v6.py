"""
Cinemática de la pierna del robot bípedo para la generación de
trayectorias (pestaña Trayectorias de la interfaz, trazador y análisis del
espacio de trabajo).

SISTEMA DE COORDENADAS (el mismo de las pestañas de cinemática inversa del
equipo, cinematica_directa_der_izq.py, en mm):
    +X a lo largo de la pierna, hacia ABAJO (vertical).
    +Y hacia adelante.
    Z lateral: la pierna izquierda está en -Z y la derecha en +Z.
    Un papel horizontal es un plano X = constante; "subir" el lápiz es
    restar a X.
    Pie con la pierna colgando:  izq (850.28, 0, -306.98)
                                 der (850.28, 0,  306.98)
Todo lo que entra y sale de este módulo (puntos, papel, posiciones) está
en ese sistema. tests/test_kinem_v6.py comprueba que posicion_pie()
coincide con cinematica_directa_der_izq.

ÁNGULOS
    Convenio de la interfaz y del URDF V9: t = (roll, pitch, rodilla) [°],
    t = 0 = pierna colgando, +pitch = pierna adelante, +rodilla = pierna
    atrás (flexión).

CÁLCULO INTERNO
    Por dentro se usa un modelo Denavit-Hartenberg del CAD V6 con
    articulación fantasma (1A2) y eslabón L6, en su propia base (el
    "modelo"), porque los métodos de cinemática inversa (geométrico,
    desacople y MTH, los del equipo) están escritos para esa tabla.
    Pierna izquierda:

        theta   d     a     alpha
  0A1    0     -L1    L2    -90°
  1A2    q1     0     0      90°   <- fantasma (cadera roll)
  2A3    0      L3    L4      0
  3A4    q2     0     L5    180°   <- cadera pitch
  4A5    q3     0     L6      0    <- rodilla

    La pierna derecha es la misma cadena con d1 = +L1 y alphas de
    0A1/1A2 cambiados de signo:
        T_der(q1, q2, q3) = Trasl(0, 0, 2·L1) · T_izq(-q1, q2, q3)
    Relación con los ángulos de la interfaz:
        izquierda:  q1 = 90° - t1    q2 = -t2    q3 = -t3
        derecha:    q1 = t1 - 90°    q2 =  t2    q3 =  t3
    y con el sistema del equipo (exacta):
        izquierda:  (x, y, z)_equipo = (-z - (L1-L2), -y, -x - (L1-L2))
        derecha:    (x, y, z)_equipo = (-z + (L1+L2),  y,  x + (L1-L2))

Los métodos de cinemática inversa devuelven TODAS las soluciones (las dos
raíces de la cadera roll y los dos codos) y se elige la que respeta los
límites del control. Sin esto, control_node recortaría el ángulo en
silencio y el pie no llegaría al punto pedido.
"""

import math

import numpy as np


# ----------------------------------------------------------------------
# Parámetros geométricos [mm]
# ----------------------------------------------------------------------

L1, L2, L3, L4, L5, L6 = 147.03, 105.1, 159.95, 94.9, 311.97, 338.31


# ----------------------------------------------------------------------
# Límites que aplica el control [°], convenio de la interfaz
# (robot_control/control_node.py y robot_teleop/teleop_node.py).
# ----------------------------------------------------------------------

LIMITES_CONTROL_DEG = ((0.0, 90.0), (-90.0, 90.0), (-90.0, 90.0))


# ----------------------------------------------------------------------
# Paso al mundo de RViz ('world' del URDF V9), en mm: el desplazamiento
# del marco del equipo a Base_link (el de teleop_node.publish_trail) y la
# rotación world -> Base_link de robot_completo.urdf.xacro (rpy 0, 90°,
# -90°). Coincide con el URDF V9 a ~2 mm.
# ----------------------------------------------------------------------

_OFFSET_BASE_LINK = {'left': np.array([8.6, 94.9, 1.5]),
                     'right': np.array([8.3, 81.6, -3.0])}
_R_MUNDO = np.array([[0.0, 1.0, 0.0],
                     [0.0, 0.0, -1.0],
                     [-1.0, 0.0, 0.0]])


# ----------------------------------------------------------------------
# Papel recomendado (scripts/espacio_trabajo/analisis_espacio_trabajo.py
# con roll >= 0°): plano X = 685.28, es decir 165 mm por encima del pie
# colgando; área segura de 255 x 255 mm con centro (Y, Z), 182.5 mm hacia
# fuera de la pierna. En toda el área (y con el lápiz levantado 20 mm)
# queda >= 5° de margen a los límites y la rodilla >= 10° de la
# singularidad.
# ----------------------------------------------------------------------

PAPEL_RECOMENDADO = {
    'left': {'x_papel': 685.28, 'centro': (0.0, -489.48), 'lado': 255.0, 'levantar': 20.0},
    'right': {'x_papel': 685.28, 'centro': (0.0, 489.48), 'lado': 255.0, 'levantar': 20.0},
}


def pose_preparacion(side='left'):
    """Punto encima del centro del papel recomendado, con el lápiz
    levantado (X menor = más arriba). La pierna colgando queda POR DEBAJO
    del papel, así que hay que llevarla aquí antes de colocar la mesa."""
    papel = PAPEL_RECOMENDADO[_lado(side)]
    y, z = papel['centro']
    return (papel['x_papel'] - papel['levantar'], y, z)


def _lado(side):
    side = str(side).lower()
    if side not in ('left', 'right'):
        raise ValueError(f"side debe ser 'left' o 'right', no {side!r}")
    return side


# ----------------------------------------------------------------------
# Conversión de ángulos
# ----------------------------------------------------------------------

def _envolver_deg(a):
    return (np.asarray(a, dtype=float) + 180.0) % 360.0 - 180.0


def interfaz_a_modelo(t_deg, side='left'):
    """t = (roll, pitch, rodilla) [°] -> q del modelo DH [rad]."""
    t1, t2, t3 = t_deg
    if _lado(side) == 'left':
        q = (90.0 - t1, -t2, -t3)
    else:
        q = (t1 - 90.0, t2, t3)
    return tuple(np.radians(v) for v in q)


def modelo_a_interfaz(q_rad, side='left'):
    """q del modelo DH [rad] -> t = (roll, pitch, rodilla) [°] en (-180, 180]."""
    q1, q2, q3 = (np.degrees(v) for v in q_rad)
    if _lado(side) == 'left':
        t = (90.0 - q1, -q2, -q3)
    else:
        t = (q1 + 90.0, q2, q3)
    return tuple(_envolver_deg(v) for v in t)


def margen_limites_deg(t_deg, limites=LIMITES_CONTROL_DEG):
    """Distancia [°] al límite más cercano (negativa si se sale)."""
    return min(m for v, (lo, hi) in zip(t_deg, limites) for m in (v - lo, hi - v))


def avisos_limites(t_deg, limites=LIMITES_CONTROL_DEG):
    nombres = ('roll', 'pitch', 'rodilla')
    return [f"{n} = {v:.1f}° fuera de [{lo:g}°, {hi:g}°]"
            for n, v, (lo, hi) in zip(nombres, t_deg, limites) if not lo <= v <= hi]


# ----------------------------------------------------------------------
# Cinemática directa
# ----------------------------------------------------------------------

def dh_matrix(theta, d, a, alpha):
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array([[ct, -st * ca, st * sa, a * ct],
                     [st, ct * ca, -ct * sa, a * st],
                     [0.0, sa, ca, d],
                     [0.0, 0.0, 0.0, 1.0]])


def fk_modelo(q_rad, side='left'):
    """MTH acumuladas (T01, T02, T03, T04, T05) para q del modelo [rad]."""
    q1, q2, q3 = q_rad
    if _lado(side) == 'left':
        A01 = dh_matrix(0.0, -L1, L2, -math.pi / 2)
        A12 = dh_matrix(q1, 0.0, 0.0, math.pi / 2)
    else:
        A01 = dh_matrix(0.0, L1, L2, math.pi / 2)
        A12 = dh_matrix(q1, 0.0, 0.0, -math.pi / 2)
    A23 = dh_matrix(0.0, L3, L4, 0.0)
    A34 = dh_matrix(q2, 0.0, L5, math.pi)
    A45 = dh_matrix(q3, 0.0, L6, 0.0)
    T01 = A01
    T02 = T01 @ A12
    T03 = T02 @ A23
    T04 = T03 @ A34
    T05 = T04 @ A45
    return T01, T02, T03, T04, T05


# ----------------------------------------------------------------------
# Cambio de sistema: modelo interno <-> sistema del equipo
# (vectorizable: p puede ser un array (..., 3))
# ----------------------------------------------------------------------

def _modelo_a_equipo(p, side='left'):
    p = np.asarray(p, dtype=float)
    x, y, z = p[..., 0], p[..., 1], p[..., 2]
    if _lado(side) == 'left':
        return np.stack([-z - (L1 - L2), -y, -x - (L1 - L2)], -1)
    return np.stack([-z + (L1 + L2), y, x + (L1 - L2)], -1)


def _equipo_a_modelo(p, side='left'):
    p = np.asarray(p, dtype=float)
    x, y, z = p[..., 0], p[..., 1], p[..., 2]
    if _lado(side) == 'left':
        return np.stack([-z - (L1 - L2), -y, -x - (L1 - L2)], -1)
    return np.stack([z - (L1 - L2), y, -x + (L1 + L2)], -1)


def posicion_pie(t_deg, side='left'):
    """Posición del pie (x, y, z) [mm], sistema del equipo, para los
    ángulos de la interfaz t [°]."""
    T = fk_modelo(interfaz_a_modelo(t_deg, side), side)[-1]
    return _modelo_a_equipo(T[:3, 3], side)


def posicion_home(side='left'):
    """Pie con la pierna colgando (todos los ángulos en 0)."""
    return posicion_pie((0.0, 0.0, 0.0), side)


def punto_a_mundo(p, side='left'):
    """Punto del sistema del equipo [mm] -> 'world' de RViz [mm]."""
    return _R_MUNDO @ (np.asarray(p, dtype=float) + _OFFSET_BASE_LINK[_lado(side)])


# ----------------------------------------------------------------------
# Cinemática inversa - pierna izquierda (métodos del equipo)
#
# Cada función devuelve TODAS las soluciones como lista de
# (q1, q2, q3) [rad] del modelo izquierdo. Son vectorizables: x, y, z
# pueden ser arrays (lo usa el análisis del espacio de trabajo); en ese
# caso cada solución trae además una máscara de validez.
# ----------------------------------------------------------------------

def _soluciones_geometrico_izq(x, y, z):
    """Método geométrico (seno/coseno explícitos)."""
    xp, zp = x - L2, z + L1
    disc = xp**2 + zp**2 - L3**2
    sols = []
    for signo_r in (1.0, -1.0):
        R = signo_r * np.sqrt(np.maximum(disc, 0.0))
        den1 = R**2 + L3**2
        q1 = np.arctan2((L3 * xp - R * zp) / den1, (R * xp + L3 * zp) / den1)
        x_arm, y_arm = R - L4, y
        c3 = (x_arm**2 + y_arm**2 - L5**2 - L6**2) / (2 * L5 * L6)
        ok = (disc >= 0) & (np.abs(c3) <= 1 + 1e-9)
        c3 = np.clip(c3, -1.0, 1.0)
        for signo_s3 in (1.0, -1.0):
            q3 = np.arctan2(signo_s3 * np.sqrt(1 - c3**2), c3)
            k1, k2 = L5 + L6 * np.cos(q3), L6 * np.sin(q3)
            den2 = k1**2 + k2**2
            q2 = np.arctan2((k2 * x_arm + k1 * y_arm) / den2, (k1 * x_arm - k2 * y_arm) / den2)
            sols.append((q1, q2, q3, ok))
    return sols


def _soluciones_desacople_izq(x, y, z):
    """Método por desacople (premultiplicación por matrices inversas)."""
    A, B, C = z + L1, x - L2, L3
    disc1 = A**2 + B**2 - C**2
    sols = []
    for signo1 in (1.0, -1.0):
        q1 = np.arctan2(B, A) - np.arctan2(signo1 * np.sqrt(np.maximum(disc1, 0.0)), C)
        c1, s1 = np.cos(q1), np.sin(q1)
        f14 = c1 * (x - L2) - s1 * (z + L1)
        Aq2, Bq2 = f14 - L4, y
        Cq2 = (Aq2**2 + Bq2**2 + L5**2 - L6**2) / (2 * L5)
        disc2 = Aq2**2 + Bq2**2 - Cq2**2
        ok = (disc1 >= -1e-9) & (disc2 >= -1e-9)
        for codo in (1.0, -1.0):
            q2 = np.arctan2(Bq2, Aq2) + codo * np.arctan2(np.sqrt(np.maximum(disc2, 0.0)), Cq2)
            c2, s2 = np.cos(q2), np.sin(q2)
            q3 = np.arctan2(s2 * Aq2 - c2 * Bq2, c2 * Aq2 + s2 * Bq2 - L5)
            sols.append((q1, q2, q3, ok))
    return sols


def _mth_izq(T):
    """Método MTH: lee los ángulos directamente de las celdas de T."""
    nx, ny, nz = T[0, 0], T[1, 0], T[2, 0]
    oy = T[1, 1]
    ax, az = T[0, 2], T[2, 2]
    px, py, pz = T[0, 3], T[1, 3], T[2, 3]
    q1 = math.atan2(-ax, -az)
    c1, s1 = math.cos(q1), math.sin(q1)
    s2 = (py - L6 * ny) / L5
    c2 = (c1 * (px - L2) - s1 * (pz + L1) - L4 - L6 * (c1 * nx - s1 * nz)) / L5
    q2 = math.atan2(s2, c2)
    q3 = q2 - math.atan2(ny, -oy)
    q3 = math.atan2(math.sin(q3), math.cos(q3))
    return q1, q2, q3


# ----------------------------------------------------------------------
# Adaptación a cualquier pierna y al convenio de la interfaz
# ----------------------------------------------------------------------

def _a_izquierda(p, side):
    """Punto de la pierna 'side' expresado para los métodos izquierdos."""
    x, y, z = p
    return (x, y, z) if side == 'left' else (x, y, z - 2 * L1)


def _q_desde_izquierda(q, side):
    q1, q2, q3 = q
    return (q1, q2, q3) if side == 'left' else (-q1, q2, q3)


def _elegir(candidatos, p, side, semilla_deg=None):
    """De las soluciones (q del modelo), devuelve la mejor en ángulos de la
    interfaz: dentro de los límites del control, con la rodilla en
    flexión (t3 >= 0) y, entre esas, la más cercana a la semilla (o la de
    mayor margen a los límites si no hay semilla)."""
    validas, fuera = [], []
    for q in candidatos:
        if not np.all(np.isfinite(q)):
            continue
        if np.linalg.norm(fk_modelo(q, side)[-1][:3, 3] - p) > 1e-6:
            continue
        t = tuple(float(v) for v in modelo_a_interfaz(q, side))
        (validas if margen_limites_deg(t) >= 0 else fuera).append(t)
    if not validas:
        motivo = 'fuera de los límites del control' if fuera else 'fuera del alcance de la pierna'
        return None, False, motivo
    flexion = [t for t in validas if t[2] >= 0] or validas
    if semilla_deg is not None:
        mejor = min(flexion, key=lambda t: max(abs(_envolver_deg(a - b)) for a, b in zip(t, semilla_deg)))
    else:
        mejor = max(flexion, key=margen_limites_deg)
    return mejor, True, ''


def _resolver(soluciones_izq, x, y, z, side, semilla_deg):
    side = _lado(side)
    p = _equipo_a_modelo((x, y, z), side)
    cand = [_q_desde_izquierda((float(a), float(b), float(c)), side)
            for a, b, c, ok in soluciones_izq(*_a_izquierda(p, side)) if bool(ok)]
    return _elegir(cand, p, side, semilla_deg)


def ik_geometrico(x, y, z, side='left', semilla_deg=None):
    """Cinemática inversa por el método geométrico. (x, y, z) en el
    sistema del equipo [mm].
    Retorna (t_deg, alcanzable, motivo): t en el convenio de la interfaz."""
    return _resolver(_soluciones_geometrico_izq, x, y, z, side, semilla_deg)


def ik_desacople(x, y, z, side='left', semilla_deg=None):
    """Cinemática inversa por desacople. Mismo formato que ik_geometrico."""
    return _resolver(_soluciones_desacople_izq, x, y, z, side, semilla_deg)


def ik_jacobiano(x, y, z, semilla_deg, side='left', tol=0.01, max_iter=300,
                 paso_max_deg=10.0, lam=1.0):
    """Cinemática inversa iterativa por el Jacobiano (mínimos cuadrados
    amortiguados), directamente sobre los ángulos de la interfaz y sin
    salirse de los límites del control. Parte de semilla_deg [°].
    Retorna (t_deg, alcanzable, motivo)."""
    side = _lado(side)
    objetivo = np.array([x, y, z], dtype=float)
    lim = np.array(LIMITES_CONTROL_DEG, dtype=float)
    t = np.clip(np.array(semilla_deg, dtype=float), lim[:, 0], lim[:, 1])
    # Con la rodilla recta la pierna está en una singularidad: el Jacobiano
    # no puede mover el pie a lo largo de la pierna y el método se atasca.
    # Se arranca con la rodilla un poco doblada.
    if abs(t[2]) < 2.0:
        t[2] = 5.0
    h = 1e-4
    for _ in range(max_iter):
        e = objetivo - posicion_pie(t, side)
        if np.linalg.norm(e) <= tol:
            return tuple(float(v) for v in t), True, ''
        J = np.empty((3, 3))
        for k in range(3):
            dt = np.zeros(3)
            dt[k] = h
            J[:, k] = (posicion_pie(t + dt, side) - posicion_pie(t - dt, side)) / (2 * h)
        dtheta = J.T @ np.linalg.solve(J @ J.T + lam**2 * np.eye(3), e)
        mayor = np.max(np.abs(dtheta))
        if mayor > paso_max_deg:
            dtheta *= paso_max_deg / mayor
        t = np.clip(t + dtheta, lim[:, 0], lim[:, 1])
    if ik_geometrico(x, y, z, side)[1]:
        return None, False, 'no convergió desde la semilla'
    return None, False, ik_geometrico(x, y, z, side)[2]


def verificar_mth(t_deg, side='left', objetivo=None, tol_deg=1e-6):
    """Verifica una solución con el método MTH: arma T = FK(t), recupera
    los ángulos leyendo solo la matriz y los compara.

    Retorna dict(p, t_recuperado, error_ang_deg, error_pos_mm, ok), con
    p la posición del pie en el sistema del equipo."""
    side = _lado(side)
    q = interfaz_a_modelo(t_deg, side)
    T = fk_modelo(q, side)[-1]
    p = _modelo_a_equipo(T[:3, 3], side)
    T_izq = T.copy()
    if side == 'right':
        T_izq[2, 3] -= 2 * L1
    q_r = _q_desde_izquierda(_mth_izq(T_izq), side)
    t_r = modelo_a_interfaz(q_r, side)
    err = max(abs(float(_envolver_deg(a - b))) for a, b in zip(t_r, t_deg))
    err_pos = None if objetivo is None else float(np.linalg.norm(p - np.asarray(objetivo, float)))
    return {'p': p, 't_recuperado': tuple(float(v) for v in t_r),
            'error_ang_deg': err, 'error_pos_mm': err_pos, 'ok': err < max(tol_deg, 1e-6)}


if __name__ == '__main__':
    for lado in ('left', 'right'):
        home = posicion_home(lado)
        print(f"{lado}: pie colgando {np.round(home, 2)} -> mundo {np.round(punto_a_mundo(home, lado), 1)}")
        objetivo = np.array(pose_preparacion(lado))
        for nombre, f in (('geométrico', ik_geometrico), ('desacople', ik_desacople)):
            t, ok, motivo = f(*objetivo, side=lado)
            print(f"  {nombre}: {np.round(t, 2) if ok else motivo}")
        t, ok, motivo = ik_jacobiano(*objetivo, (0.0, 0.0, 0.0), side=lado)
        print(f"  jacobiano: {np.round(t, 2) if ok else motivo}")
