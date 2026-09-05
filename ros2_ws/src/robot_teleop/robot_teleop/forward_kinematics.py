"""Cinematica directa de la pata mediante Denavit-Hartenberg."""

import math


def _matmul(left, right):
    """Multiplica dos matrices 4x4 representadas como listas."""
    return [
        [sum(left[row][k] * right[k][col] for k in range(4))
         for col in range(4)]
        for row in range(4)
    ]


def dh_matrix(theta, d, a, alpha):
    """Devuelve la transformacion homogenea DH de un eslabon."""
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return [
        [ct, -st * ca, st * sa, a * ct],
        [st, ct * ca, -ct * sa, a * st],
        [0.0, sa, ca, d],
        [0.0, 0.0, 0.0, 1.0],
    ]


def end_effector_matrix(joint_angles_deg, link_lengths=None):
    """
    Calcula 0A4 para los tres angulos de los sliders, dados en grados.

    La cadena conserva la tabla DH usada originalmente en ``kinem_leg_gen``.
    Las cinco longitudes pueden configurarse; por defecto valen una unidad.
    """
    if len(joint_angles_deg) != 3:
        raise ValueError('La cinematica de la pata requiere exactamente 3 angulos')

    lengths = [1.0] * 5 if link_lengths is None else list(link_lengths)
    if len(lengths) != 5:
        raise ValueError('Se requieren exactamente 5 longitudes de eslabon')

    l1, l2, l3, l4, l5 = lengths
    q1, q2, q3 = (math.radians(value) for value in joint_angles_deg)

    transforms = (
        dh_matrix(0.0, d=l1, a=l2, alpha=math.pi / 2.0),
        dh_matrix(q1, d=0.0, a=l3, alpha=-math.pi / 2.0),
        dh_matrix(q2, d=0.0, a=l4, alpha=math.pi),
        dh_matrix(q3, d=0.0, a=l5, alpha=0.0),
    )

    result = transforms[0]
    for transform in transforms[1:]:
        result = _matmul(result, transform)
    return result
