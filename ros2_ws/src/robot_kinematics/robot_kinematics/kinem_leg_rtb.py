"""
Cinemática directa
usando la librería roboticstoolbox.
"""

import roboticstoolbox as rtb
import numpy as np

# ----------------------------------------------------------------------
# Parámetros geométricos (longitudes de eslabones)
# ----------------------------------------------------------------------
L1, L2, L3, L4, L5 = 1.0, 1.0, 1.0, 1.0, 1.0

# Ángulos articulares en grados
q1_deg = 100
q2_deg = 0
q3_deg = 0

# Límites articulares (en grados) 
q1_min, q1_max = -40, 40
q2_min, q2_max = -60, 60
q3_min, q3_max = -70, 70

# Función para limitar
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

# ----------------------------------------------------------------------
# Modelo DH
# ----------------------------------------------------------------------
robot = rtb.DHRobot([
    rtb.RevoluteDH(d=L1, a=L2, alpha=np.pi/2,  qlim=[0, 0]),      # articulación fija
    rtb.RevoluteDH(d=0,  a=L3, alpha=-np.pi/2, qlim=np.radians([q1_min, q1_max])),
    rtb.RevoluteDH(d=0,  a=L4, alpha=np.pi,    qlim=np.radians([q2_min, q2_max])),
    rtb.RevoluteDH(d=0,  a=L5, alpha=0,        qlim=np.radians([q3_min, q3_max])),
], name='pata')

print(robot)

# ----------------------------------------------------------------------
# Cálculo de matrices de transformación
# ----------------------------------------------------------------------
q = np.radians([0, q1_deg, q2_deg, q3_deg])   # vector articular en radianes

# fkine_all devuelve todas las transformaciones acumuladas desde la base
Ts = robot.fkine_all(q)

# Mostramos las matrices 0A1, 0A2, 0A3, 0A4
nombres = ['0A1', '0A2', '0A3', '0A4']
print(f"\n{'='*50}")
print(f"q1 = {q1_deg}°  q2 = {q2_deg}°  q3 = {q3_deg}°")
print(f"{'='*50}")
for nombre, T in zip(nombres, Ts[1:]):
    print(f"\n{nombre} =")
    print(np.round(T.A, 4))