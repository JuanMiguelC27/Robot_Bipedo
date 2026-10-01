"""
Cinemática inversa
Implementación algebraica (roll-pitch-pitch con offset DH).
"""

import numpy as np

L1, L2, L3, L4, L5 = 1, 1, 1, 1, 1

x, y, z = 4, 0, 1

# Límites articulares [°]
q1_min, q1_max = -160, 70
q2_min, q2_max = -115, 115
q3_min, q3_max = -85, 65

def cinematica_inversa_pata_alg(x, y, z, L1, L2, L3, L4, L5, codo="arriba"):
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

q1, q2, q3, alcanzable = cinematica_inversa_pata_alg(x, y, z, L1, L2, L3, L4, L5, codo="arriba")

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
