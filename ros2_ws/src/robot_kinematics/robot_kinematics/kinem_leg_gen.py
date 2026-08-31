"""
Cinemática directa 
Implementación manual con Denavit-Hartenberg.
"""

import numpy as np

# ----------------------------------------------------------------------
# Parámetros geométricos (longitudes de eslabones)
# ----------------------------------------------------------------------
L1, L2, L3, L4, L5 = 1.0, 1.0, 1.0, 1.0, 1.0

# Ángulos articulares en grados
q1_deg = 0   
q2_deg = 0
q3_deg = 0

# Límites articulares (en grados)
q1_min, q1_max = -40, 40
q2_min, q2_max = -60, 60
q3_min, q3_max = -70, 70

# ----------------------------------------------------------------------
# Función para limitar un valor entre mn y mx
# ----------------------------------------------------------------------
def clamp(v, mn, mx):
    return max(mn, min(v, mx))

# Guardar originales para avisar
q1_orig, q2_orig, q3_orig = q1_deg, q2_deg, q3_deg

# Aplicar límites
q1_deg = clamp(q1_deg, q1_min, q1_max)
q2_deg = clamp(q2_deg, q2_min, q2_max)
q3_deg = clamp(q3_deg, q3_min, q3_max)

# Si el ángulo original estaba fuera de rango esto indica que se ajustó
if q1_deg != q1_orig:
    print(f"Advertencia: q1 fuera de rango, ajustado a {q1_deg}°")
if q2_deg != q2_orig:
    print(f"Advertencia: q2 fuera de rango, ajustado a {q2_deg}°")
if q3_deg != q3_orig:
    print(f"Advertencia: q3 fuera de rango, ajustado a {q3_deg}°")

# Conversión a radianes
q1 = np.radians(q1_deg)
q2 = np.radians(q2_deg)
q3 = np.radians(q3_deg)

# ----------------------------------------------------------------------
# Función que construye la matriz DH individual
# ----------------------------------------------------------------------
def dh_matrix(theta, d, a, alpha):
    """
    Calcula la matriz de transformación homogénea 4x4
    según los parámetros de Denavit-Hartenberg.
    """
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)

    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0,   sa,     ca,    d   ],
        [0,   0,      0,     1   ]
    ])

# ----------------------------------------------------------------------
# Matrices individuales según la tabla DH
# ----------------------------------------------------------------------
# Articulación 1 (fija, theta=0)
A01 = dh_matrix(0, d=L1, a=L2, alpha=np.pi/2)

# Articulación 2 (variable q1)
A12 = dh_matrix(q1, d=0, a=L3, alpha=-np.pi/2)

# Articulación 3 (variable q2)
A23 = dh_matrix(q2, d=0, a=L4, alpha=np.pi)

# Articulación 4 (variable q3)
A34 = dh_matrix(q3, d=0, a=L5, alpha=0)

# ----------------------------------------------------------------------
# Matrices acumuladas (cinemática directa)
# ----------------------------------------------------------------------
A02 = A01 @ A12
A03 = A02 @ A23
A04 = A03 @ A34

# ----------------------------------------------------------------------
# Mostrar resultados
# ----------------------------------------------------------------------
def imprimir_matriz(nombre, M):
    print(f"\n{nombre} =")
    print(np.round(M, 4))

print(f"{'='*50}")
print(f"q1 = {q1_deg}°  q2 = {q2_deg}°  q3 = {q3_deg}°")
print(f"{'='*50}")

imprimir_matriz('0A1', A01)
imprimir_matriz('0A2', A02)
imprimir_matriz('0A3', A03)
imprimir_matriz('0A4', A04)