"""
Cinemática inversa de la pata del robot bípedo.

Implementación geométrica (seno/coseno explícitos, roll-pitch-pitch
con offset DH). Modelo nuevo: con articulación fantasma (1A2) y
eslabón L6. Pata IZQUIERDA (tabla corregida: alpha=-90° en 0A1 y en 1A2).
"""

import numpy as np

L1, L2, L3, L4, L5, L6 = 147.03, 105.1, 159.95, 94.9, 311.97, 338.31

x, y, z = 850.28, 0.0, -306.98   # objetivo de prueba (q1=q2=q3=0)

# Límites articulares [°]
q1_min, q1_max = -0, 90
q2_min, q2_max = --90, 90
q3_min, q3_max = --90, 90


def cinematica_inversa_pata_geom(x, y, z, L1, L2, L3, L4, L5, L6, codo="arriba"):
    """
    Resuelve q1, q2, q3 [rad] por el método geométrico (plano XZ para
    la cadera roll, plano del "brazo" para rodilla y cadera pitch),
    usando seno/coseno en vez de atan2 con offset.

    codo="arriba" -> sin(q3) negativo
    codo="abajo"  -> sin(q3) positivo

    Retorna (q1, q2, q3, alcanzable).
    """
    # Paso 1: variables trasladadas y alcance R (plano XZ)
    xp = x - L2
    zp = z + L1                                    # <- z + L1, no z - L1
    disc = xp**2 + zp**2 - L3**2
    if disc < 0:
        return None, None, None, False
    R = np.sqrt(disc)

    # Paso 2: q1
    denom1 = R**2 + L3**2
    c1 = (R*xp - L3*zp) / denom1                    # <- signos ajustados
    s1 = -(R*zp + L3*xp) / denom1                    # <- (tabla corregida)
    q1 = np.arctan2(s1, c1)

    # Paso 3: posición del "brazo" (plano XY) y q3
    x_arm = R - L4
    y_arm = -y                                       # <- -y, no y

    c3 = (x_arm**2 + y_arm**2 - L5**2 - L6**2) / (2*L5*L6)
    if abs(c3) > 1 + 1e-9:
        return None, None, None, False
    c3 = np.clip(c3, -1.0, 1.0)

    s3 = np.sqrt(1 - c3**2) * (-1 if codo == "arriba" else 1)
    q3 = np.arctan2(s3, c3)

    # Paso 4: q2
    k1 = L5 + L6*np.cos(q3)
    k2 = L6*np.sin(q3)
    denom2 = k1**2 + k2**2
    c2 = (k1*x_arm - k2*y_arm) / denom2
    s2 = (k2*x_arm + k1*y_arm) / denom2
    q2 = np.arctan2(s2, c2)

    return q1, q2, q3, True


q1, q2, q3, alcanzable = cinematica_inversa_pata_geom(x, y, z, L1, L2, L3, L4, L5, L6, codo="arriba")

print(f"{'='*50}")
print(f"Objetivo: x = {x}  y = {y}  z = {z}")
print(f"{'='*50}")

if not alcanzable:
    print("No alcanzable")
else:
    q1d, q2d, q3d = np.degrees([q1, q2, q3])
    print(f"q1 = {q1d:.2f}°  q2 = {q2d:.2f}°  q3 = {q3d:.2f}°")
    for nombre, v, mn, mx in (("q1", q1d, q1_min, q1_max),
                              ("q2", q2d, q2_min, q2_max),
                              ("q3", q3d, q3_min, q3_max)):
        if not mn <= v <= mx:
            print(f"Advertencia: {nombre} fuera de rango [{mn}°, {mx}°]")
