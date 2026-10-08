"""Pruebas de perfiles_temporales con los ejemplos resueltos de la clase
"Manipulator Inverse kinematics II" (diapositivas 47-73)."""
import math

import numpy as np
import pytest

from robot_kinematics import perfiles_temporales as pt


def coef_global(perfil, k=0):
    return perfil.coeficientes_globales()[k][2]


def test_cubico_5_a_25_en_2s():                       # diap. 47
    p = pt.cubico(5, 25, 0, 2)
    assert np.allclose(coef_global(p), [5, 0, 15, -5])


def test_cubico_de_t2_a_t4():                         # diap. 49
    p = pt.cubico(10, 60, 2, 4)
    assert np.allclose(coef_global(p), [260, -300, 112.5, -12.5])
    _, v, a = p.evaluar(np.array([2.0, 3.0, 4.0]))
    assert np.allclose(v, [0, 37.5, 0])
    assert math.isclose(a[0], 75) and math.isclose(a[-1], -75)


def test_quintico_de_t2_a_t4():                       # diap. 51
    p = pt.quintico(10, 60, 2, 4)
    assert np.allclose(coef_global(p), [-1540, 3000, -2250, 812.5, -140.625, 9.375])
    _, v, a = p.evaluar(np.array([2.0, 4.0]))
    assert np.allclose(v, 0) and np.allclose(a, 0)     # sin saltos de aceleración


def test_trapezoidal_con_vmax():                      # diap. 58
    p = pt.trapezoidal(10, 60, 0, 4, vmax=20)
    assert math.isclose(p.info['tb'], 1.5)
    q, v, a = p.evaluar(np.array([1.0, 2.0, 3.5]))
    assert np.allclose(q, [10 + 20 / 3 * 1, -5 + 40, 60 - 20 / 3 * 0.25])
    assert math.isclose(v[1], 20) and math.isclose(a[0], 40 / 3)


def test_trapezoidal_limites_de_vmax():
    with pytest.raises(pt.ErrorPerfil):
        pt.trapezoidal(10, 60, 0, 4, vmax=12)         # < (qf-q0)/tf
    with pytest.raises(pt.ErrorPerfil):
        pt.trapezoidal(10, 60, 0, 4, vmax=26)         # > 2(qf-q0)/tf


def test_lineal_con_ajuste_parabolico():              # diap. 61
    p = pt.trapezoidal(5, 25, 0, 2, amax=40)
    assert math.isclose(p.info['tb'], 0.2929, abs_tol=1e-3)
    assert math.isclose(p.info['vmax'], 11.71, abs_tol=0.01)
    q, _, _ = p.evaluar(np.array([p.info['tb'], 2.0]))
    assert math.isclose(q[0], 6.716, abs_tol=0.01) and math.isclose(q[1], 25)


def test_tiempo_minimo():                             # diap. 60
    p = pt.tiempo_minimo(10, 60, 0, 13.33)
    assert math.isclose(p.tf, 3.8735, abs_tol=1e-3)
    assert math.isclose(p.info['ts'], 1.937, abs_tol=1e-3)
    q, _, _ = p.evaluar(np.array([p.tf]))
    assert math.isclose(q[0], 60)


def test_cubico_con_punto_intermedio():               # diap. 65
    p = pt.cubico_puntos([5, 15, 20], [0, 1, 2], vs=[0, 5, 0])
    assert np.allclose(p.tramos[0][2], [5, 0, 25, -15])
    assert np.allclose(p.tramos[1][2], [15, 5, 5, -5])


def test_velocidades_de_paso():                       # diap. 64
    vs = pt.velocidades_de_paso([0, 2, 6, 3, 1], [0, 1, 2, 3, 4])
    assert vs == [0.0, 3.0, 0.0, -2.5, 0.0]


def test_parabolico_por_puntos():                     # diap. 70-73
    p = pt.parabolico_puntos([5, 20, 15, 5], [0, 2, 3, 4], 30)
    i = p.info
    assert np.allclose(i['tb'], [0.268, 0.435, 0.256, 0.423], atol=1e-3)
    assert np.allclose(i['velocidades'], [8.04, -5.0, -12.68], atol=1e-2)
    # El ejemplo de la diap. 73 da t34 = 0.660 usando la fórmula de los tramos
    # interiores (0.5·tp4); la fórmula del tramo final de la diap. 69 (y del
    # libro de Craig) usa la mezcla final completa: 1 - 0.423 - 0.5·0.256 = 0.449.
    # Con 0.660 la curva no terminaría en reposo en 5°.
    assert np.allclose(i['tiempos_lineales'], [1.514, 0.654, 0.449], atol=2e-3)
    q, v, _ = p.evaluar(np.array([0.0, 4.0]))
    assert np.allclose(q, [5, 5], atol=1e-9) and np.allclose(v, 0, atol=1e-9)
    # continuidad de posición y velocidad entre tramos
    for (_, b, _), (c, _, _) in zip(p.tramos, p.tramos[1:]):
        q1, v1, _ = p.evaluar(np.array([b - 1e-7, c + 1e-7]))
        assert abs(q1[0] - q1[1]) < 1e-5 and abs(v1[0] - v1[1]) < 1e-4


def test_ptp_coordinado_tiempo_minimo():
    perf = pt.planificar_ptp([0, 0, 0], [40, 10, -20], 'tiempo_minimo', params={'amax': 50})
    assert len({round(p.tf, 9) for p in perf}) == 1      # todas terminan a la vez
    libres = pt.planificar_ptp([0, 0, 0], [40, 10, -20], 'tiempo_minimo', params={'amax': 50}, coordinado=False)
    assert len({round(p.tf, 6) for p in libres}) == 3


def test_ley_temporal_camino():
    pts = np.array([[0, 0, 0], [30, 0, 0], [30, 40, 0]], float)
    t, P = pt.ley_temporal_camino(pts, 'trapezoidal', 20.0, 100.0)
    v = np.linalg.norm(np.diff(P, axis=0), axis=1) / np.diff(t)
    assert v.max() <= 20.0 + 1e-6
    assert np.allclose(P[0], pts[0]) and np.allclose(P[-1], pts[-1])
    t2, P2 = pt.ley_temporal_camino(pts, 'quintico', 20.0)
    v2 = np.linalg.norm(np.diff(P2, axis=0), axis=1) / np.diff(t2)
    assert v2.max() <= 20.0 + 0.1
