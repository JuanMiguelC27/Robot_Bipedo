"""
Cinemática inversa
Implementación algebraica (roll-pitch-pitch con offset DH).
Modelo nuevo: con articulación fantasma (1A2) y eslabón L6.
"""

import numpy as np

L1, L2, L3, L4, L5, L6 = 147.03, 105.1, 159.95, 94.9, 311.97, 338.31

x, y, z = 850.28, 0, 306.98   # objetivo de prueba (q1=q2=q3=0 con L=1)

# Límites articulares [°]
q1_min, q1_max = 172, 270
q2_min, q2_max = 45, 225
q3_min, q3_max = 45, 225

def cinematica_inversa_pata_alg(x, y, z, L1, L2, L3, L4, L5, L6, codo="arriba"):
    xp = x - L2
    zp = z - L1
    rho2 = xp**2 + zp**2

    disc = rho2 - L3**2
    if disc < 0:
        return None, None, None, False
    r = np.sqrt(disc)

    q1 = np.arctan2(zp, xp) - np.arctan2(L3, r)

    u = r - L4
    v = y

    c3 = (u**2 + v**2 - L5**2 - L6**2) / (2*L5*L6)
    if abs(c3) > 1 + 1e-9:                 # tolerancia numérica en el borde
        return None, None, None, False
    c3 = np.clip(c3, -1.0, 1.0)

    s3 = np.sqrt(1 - c3**2) * (1 if codo == "arriba" else -1)
    q3 = np.arctan2(s3, c3)

    A = L5 + L6*np.cos(q3)
    B = L6*np.sin(q3)
    det = A**2 + B**2
    c2 = (A*u - B*v) / det
    s2 = (B*u + A*v) / det
    q2 = np.arctan2(s2, c2)

    return q1, q2, q3, True

q1, q2, q3, alcanzable = cinematica_inversa_pata_alg(x, y, z, L1, L2, L3, L4, L5, L6, codo="arriba")

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
            
