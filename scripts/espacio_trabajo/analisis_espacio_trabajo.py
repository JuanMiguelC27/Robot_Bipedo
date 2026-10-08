#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Análisis del espacio de trabajo, singularidades y superficie horizontal
óptima de escritura para una pierna del robot bípedo (CAD V6); izquierda
por defecto.

Usa la cinemática de robot_kinematics/kinem_v6.py, la misma que la
pestaña Trayectorias de la interfaz y el trazador:

- resultados en el sistema de las pestañas de cinemática inversa del
  equipo: +X a lo largo de la pierna hacia abajo, Y adelante, Z lateral;
  un papel horizontal es un plano X = constante. (Por dentro el cálculo
  se hace en la base del modelo DH, donde la vertical es su eje Z.)
- ángulos en el convenio de la interfaz (t = 0 = pierna colgando);
- límites del control (kinem_v6.LIMITES_CONTROL_DEG).

Uso:
    python3 analisis_espacio_trabajo.py [--pierna left|right] [--salida DIR]
Genera las figuras en DIR (por defecto docs/img/espacio_trabajo_v6/) e
imprime las tablas que recoge docs/ESPACIO_TRABAJO_Y_SUPERFICIE_ESCRITURA.md.
Solo sobrescribe las figuras que genera; no borra nada más de la carpeta.
"""
import argparse
import math
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

REPO = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
sys.path.insert(0, os.path.join(REPO, "ros2_ws", "src", "robot_kinematics"))

from robot_kinematics import kinem_v6 as K   # noqa: E402

_args = argparse.ArgumentParser(description=__doc__.splitlines()[1])
_args.add_argument("--pierna", choices=("left", "right"), default="left")
_args.add_argument("--salida", default=os.path.join(REPO, "docs", "img", "espacio_trabajo_v6"))
_args = _args.parse_args()

OUT = _args.salida
SIDE = _args.pierna

LIM_CTRL = np.array(K.LIMITES_CONTROL_DEG)
# Comparación: cadera roll hasta -20° (nominal del URDF V6) en lugar del
# límite inferior del control.
LIM_URDF = LIM_CTRL.copy()
LIM_URDF[0, 0] = -20.0

STEP = 5.0                        # resolución de las rejillas [mm]
LIFT = 20.0                       # elevación del lápiz entre figuras [mm]
Q3_SAFE = 10.0                    # distancia mínima a la singularidad rodilla recta [°]
MARGIN_SAFE = 5.0                 # margen mínimo a los límites articulares [°]

# Por dentro se trabaja en la base del modelo DH (Z vertical hacia arriba);
# todo lo que se imprime o se dibuja se pasa al sistema del equipo.
def G(p):
    """Punto(s) del modelo -> sistema del equipo."""
    return K._modelo_a_equipo(p, SIDE)


R_G = G(np.eye(3)) - G(np.zeros(3))   # filas: imagen de los ejes del modelo
R_G = R_G.T                           # matriz de rotación modelo -> equipo


# ----------------------------------------------------------------------------
#  Cinemática vectorizada (mismas fórmulas que kinem_v6, sobre arrays)
# ----------------------------------------------------------------------------

def _dh(theta, d, a, alpha):
    theta = np.asarray(theta, float)
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    z, o = np.zeros_like(theta), np.ones_like(theta)
    return np.stack([np.stack([ct, -st * ca, st * sa, a * ct], -1),
                     np.stack([st, ct * ca, -ct * sa, a * st], -1),
                     np.stack([z, z + sa, z + ca, z + d], -1),
                     np.stack([z, z, z, o], -1)], -2)


def fk(t1, t2, t3):
    """Posición del pie [mm] para ángulos de la interfaz [°] (arrays)."""
    q1, q2, q3 = K.interfaz_a_modelo((t1, t2, t3), SIDE)
    # Pierna derecha = cadena izquierda con -q1, desplazada 2·L1 en Z.
    if SIDE == "right":
        q1 = -q1
    T = (_dh(0 * q1, -K.L1, K.L2, -np.pi / 2) @ _dh(q1, 0, 0, np.pi / 2) @ _dh(0 * q1, K.L3, K.L4, 0)
         @ _dh(q2, 0, K.L5, np.pi) @ _dh(q3, 0, K.L6, 0))
    dz = 2 * K.L1 if SIDE == "right" else 0.0
    return np.stack([T[..., 0, 3], T[..., 1, 3], T[..., 2, 3] + dz], -1)


def jac(t1, t2, t3, h=1e-4):
    """Jacobiano numérico [mm/°] respecto a los ángulos de la interfaz."""
    cols = []
    for k in range(3):
        tp = [t1, t2, t3]
        tm = [t1, t2, t3]
        tp[k] = tp[k] + h
        tm[k] = tm[k] - h
        cols.append((fk(*tp) - fk(*tm)) / (2 * h))
    return np.stack(cols, -1)


def inv_cond(t, rows=(0, 1, 2)):
    """1/κ del Jacobiano (o de sus filas): 1 = isotrópico, 0 = singular.
    rows=(0, 1) mide solo el movimiento dentro del papel (X, Y)."""
    s = np.linalg.svd(jac(*t)[..., list(rows), :], compute_uv=False)
    return s[..., -1] / s[..., 0]


def margin_deg(t, lim):
    return np.minimum.reduce([m for v, (lo, hi) in zip(t, lim) for m in (v - lo, hi - v)])


def check_against_module():
    """La versión vectorizada coincide con kinem_v6."""
    rng = np.random.default_rng(0)
    for _ in range(200):
        t = rng.uniform(LIM_CTRL[:, 0], LIM_CTRL[:, 1])
        assert np.allclose(G(fk(*t)), K.posicion_pie(t, SIDE), atol=1e-6)


# ----------------------------------------------------------------------------
#  Plano horizontal Z = h  (papel en X-Y)
# ----------------------------------------------------------------------------

HANG = fk(0.0, 0.0, 0.0)          # pie con la pierna colgando (modelo)
XS = HANG[0] + np.arange(-450, 450 + STEP, STEP)
YS = np.arange(-450, 450 + STEP, STEP)
GX, GY = np.meshgrid(XS, YS)          # filas: Y, columnas: X
_cache = {}


def plane(h, lim):
    """Rejilla del plano Z = h. Un punto vale si alguna solución de la IK
    respeta los límites; se queda con la de mayor margen."""
    key = (float(h), lim.tobytes())
    if key in _cache:
        return _cache[key]
    Z = np.full_like(GX, h)
    best_m = np.full(GX.shape, -np.inf)
    best_t = [np.zeros(GX.shape) for _ in range(3)]
    dz = 2 * K.L1 if SIDE == "right" else 0.0
    for q1, q2, q3, ok in K._soluciones_geometrico_izq(GX, GY, Z - dz):
        t = K.modelo_a_interfaz(((-q1 if SIDE == "right" else q1), q2, q3), SIDE)
        m = np.where(ok, margin_deg(t, lim), -np.inf)
        better = m > best_m
        best_m = np.where(better, m, best_m)
        for k in range(3):
            best_t[k] = np.where(better, t[k], best_t[k])
    good = best_m >= 0
    t = [np.where(good, v, 0.0) for v in best_t]
    knee = np.abs(t[2])
    safe = good & (knee >= Q3_SAFE) & (best_m >= MARGIN_SAFE)
    _cache[key] = dict(good=good, safe=safe, t=t,
                       icp=np.where(good, inv_cond(t, rows=(0, 1)), np.nan),
                       ic=np.where(good, inv_cond(t), np.nan),
                       margin=np.where(good, best_m, np.nan), knee=np.where(good, knee, np.nan))
    return _cache[key]


def largest_square(mask):
    """Mayor cuadrado (alineado con X, Y) sin huecos en 'mask'.
    Devuelve (lado [mm], X centro, Y centro, máscara). Entre cuadrados
    iguales, el más cercano a la vertical del pie colgando."""
    n, m = mask.shape
    dp = np.zeros((n, m), int)
    for i in range(n):
        for j in range(m):
            if mask[i, j]:
                dp[i, j] = 1 if i == 0 or j == 0 else 1 + min(dp[i - 1, j], dp[i, j - 1], dp[i - 1, j - 1])
    k = dp.max()
    if k < 2:
        return 0.0, np.nan, np.nan, np.zeros_like(mask)
    best = None
    for i, j in zip(*np.nonzero(dp == k)):
        xc, yc = (XS[j] + XS[j - k + 1]) / 2, (YS[i] + YS[i - k + 1]) / 2
        d = math.hypot(xc - HANG[0], yc - HANG[1])
        if best is None or d < best[0]:
            best = (d, i, j, xc, yc)
    _, i, j, xc, yc = best
    sq = np.zeros_like(mask)
    sq[i - k + 1:i + 1, j - k + 1:j + 1] = True
    return (k - 1) * STEP, xc, yc, sq


def analyze(h, lim, lift=LIFT):
    p, pl = plane(h, lim), plane(h + lift, lim)
    side0 = largest_square(p["good"])[0]
    side = largest_square(p["good"] & pl["good"])[0]
    side_s, xc, yc, sq = largest_square(p["safe"] & pl["safe"])
    if not sq.any():
        sq = largest_square(p["good"] & pl["good"])[3]
    f = (lambda a, fn: fn(a[sq]) if sq.any() else np.nan)
    return dict(h=h, above=h - HANG[2], area=p["good"].sum() * STEP**2 / 100,
                side_nolift=side0, side=side, side_safe=side_s, xc=xc, yc=yc,
                icp_min=f(p["icp"], np.nanmin), ic_min=f(p["ic"], np.nanmin),
                margin=f(p["margin"], np.nanmin), knee_min=f(p["knee"], np.nanmin),
                t1=(f(p["t"][0], np.nanmin), f(p["t"][0], np.nanmax)), sq=sq)


# ----------------------------------------------------------------------------
#  Figuras
# ----------------------------------------------------------------------------

def fig_workspace(h_rec):
    """Cortes del espacio de trabajo con los límites del control, en el
    sistema del equipo (X hacia abajo: el eje vertical está invertido)."""
    n = 400
    x_papel = G([0.0, 0.0, h_rec])[0]
    fig, ax = plt.subplots(1, 2, figsize=(12, 5.8))
    # sagital: roll en 0, pitch y rodilla libres -> plano Y-X
    t2, t3 = np.meshgrid(np.linspace(*LIM_CTRL[1], n), np.linspace(*LIM_CTRL[2], n))
    t1 = np.zeros_like(t2)
    p = G(fk(t1, t2, t3))
    det = np.abs(np.linalg.det(jac(t1, t2, t3)))
    sc = ax[0].scatter(p[..., 1], p[..., 0], c=det / det.max(), s=1, cmap="viridis", rasterized=True)
    sing = np.abs(t3) < 0.6
    ax[0].scatter(p[..., 1][sing], p[..., 0][sing], s=2, c="red", label="singularidad: rodilla recta")
    # frontal: roll en todo su rango -> plano Z-X
    zs, xs = [], []
    g2, g3 = np.meshgrid(np.linspace(*LIM_CTRL[1], 120), np.linspace(*LIM_CTRL[2], 120))
    for r in np.linspace(*LIM_CTRL[0], 400):
        pf = G(fk(np.full_like(g2, r), g2, g3))
        zs.append(pf[..., 2].ravel())
        xs.append(pf[..., 0].ravel())
    ax[1].scatter(np.concatenate(zs), np.concatenate(xs), s=0.3, c="#1f6fd1", alpha=0.2, rasterized=True)
    hang = G(HANG)
    for a, horiz in ((ax[0], hang[1]), (ax[1], hang[2])):
        a.axhline(x_papel, color="k", lw=1)
        a.axhline(x_papel - LIFT, color="k", ls="--", lw=1)
        a.plot(horiz, hang[0], "o", mfc="none", mec="k", ms=8)
        a.plot(0, 0, "k^", ms=8)
        a.set_aspect("equal")
        a.invert_yaxis()
    ax[0].text(ax[0].get_xlim()[1], x_papel + 8, f"papel X = {x_papel:.0f}", fontsize=8, va="top", ha="right")
    ax[0].text(15, 0, "origen (cadera)", fontsize=8, va="center")
    ax[0].set(xlabel="Y [mm]  (+ = adelante)", ylabel="X [mm]  (+ = abajo)",
              title="Corte sagital (roll = 0) con los límites del control")
    ax[0].legend(loc="lower right", fontsize=8)
    fig.colorbar(sc, ax=ax[0], label="|det J| normalizado  (0 = singular)", shrink=0.8)
    ax[1].set(xlabel="Z [mm]  (lateral)", ylabel="X [mm]  (+ = abajo)",
              title=f"Corte frontal (roll {LIM_CTRL[0][0]:g}°…{LIM_CTRL[0][1]:g}°)")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "espacio_trabajo_cortes.png"), dpi=110)
    plt.close(fig)


def fig_planes(rows, rows_urdf, h_rec):
    a = [r["above"] for r in rows]
    fig, ax = plt.subplots(1, 2, figsize=(12, 4.4))
    ax[0].plot(a, [r["side_safe"] for r in rows_urdf], ":", color="#999",
               label="seguro, con roll ≥ -20° (límite del URDF)")
    ax[0].plot(a, [r["side_nolift"] for r in rows], "o-", ms=4, label="solo el papel")
    ax[0].plot(a, [r["side"] for r in rows], "s-", ms=4, label=f"+ lápiz levantado {LIFT:g} mm")
    ax[0].plot(a, [r["side_safe"] for r in rows], "^-", ms=4,
               label=f"seguro: rodilla ≥ {Q3_SAFE:g}°, margen ≥ {MARGIN_SAFE:g}°")
    ax[0].axvline(h_rec - HANG[2], color="k", lw=1)
    ax[0].set(xlabel="Altura del papel sobre el pie colgando [mm]", ylabel="Lado del mayor cuadrado [mm]",
              title="Área de escritura según la altura del papel")
    ax[0].legend(fontsize=7.5, loc="lower right")
    ax[0].grid(alpha=0.3)
    ax[1].plot(a, [r["icp_min"] for r in rows], "o-", ms=4, label="1/κ en el papel (Y, Z): mínimo")
    ax[1].plot(a, [r["ic_min"] for r in rows], "s-", ms=4, label="1/κ en 3D (incluye subir/bajar): mínimo")
    ax[1].set_ylim(0, 1.05)
    ax[1].axvline(h_rec - HANG[2], color="k", lw=1)
    ax[1].set(xlabel="Altura del papel sobre el pie colgando [mm]", ylabel="1/κ(J)  (1 = ideal)",
              title="Calidad del movimiento en el cuadrado seguro")
    ax[1].legend(fontsize=8, loc="lower left")
    ax[1].grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "comparacion_planos.png"), dpi=110)
    plt.close(fig)


def fig_plane_map(rw):
    """Mapa del papel visto desde arriba, en el sistema del equipo:
    horizontal Z (lateral), vertical Y (adelante)."""
    h = rw["h"]
    p = plane(h, LIM_CTRL)
    GG = G(np.stack([GX, GY, np.full_like(GX, h)], -1))
    gz, gy = GG[..., 2], GG[..., 1]
    hang = G(HANG)
    fig, ax = plt.subplots(1, 2, figsize=(12, 5.4))
    im = ax[0].pcolormesh(gz, gy, p["icp"], cmap="viridis", vmin=0, vmax=1, shading="auto")
    fig.colorbar(im, ax=ax[0], label="1/κ del movimiento en el papel (Y, Z)", shrink=0.85)
    im2 = ax[1].pcolormesh(gz, gy, p["margin"], cmap="magma", vmin=0, vmax=60, shading="auto")
    fig.colorbar(im2, ax=ax[1], label="margen al límite articular más cercano [°]", shrink=0.85)
    lost = plane(h, LIM_URDF)["good"] & ~p["good"]
    for a, t in zip(ax, ("Manipulabilidad", "Margen a los límites")):
        if lost.any():
            a.contourf(gz, gy, lost.astype(float), levels=[0.5, 1.5], colors="none", hatches=["////"])
        a.plot(hang[2], hang[1], "k+", ms=12)
        if rw["side_safe"] > 0:
            s = rw["side_safe"] / 2
            c = G([rw["xc"], rw["yc"], h])
            a.add_patch(plt.Rectangle((c[2] - s, c[1] - s), 2 * s, 2 * s,
                                      fill=False, ec="#2ca02c", lw=2.5))
            a.text(c[2] - s, c[1] + s + 8,
                   f"seguro {rw['side_safe']:.0f} × {rw['side_safe']:.0f} mm", fontsize=8,
                   color="#1b7a1b", bbox=dict(fc="white", ec="none", alpha=0.85, pad=1.5))
        a.set(xlabel="Z [mm]  (lateral)", ylabel="Y [mm]  (+ = adelante)",
              title=f"{t} (vista desde arriba)\npapel X = {G([0, 0, h])[0]:.0f}, "
                    f"{rw['above']:.0f} mm sobre el pie colgando")
        a.set_xlim(hang[2] - 400, hang[2] + 400)
        a.set_ylim(-400, 400)
        a.set_aspect("equal")
    ax[0].text(hang[2] - 390, -390, f"+ : pie colgando.  Rayado: lo recorta el control (roll < {LIM_CTRL[0, 0]:g}°)",
               fontsize=7.5, va="bottom")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, f"plano_{rw['above']:.0f}mm.png"), dpi=110)
    plt.close(fig)


# ----------------------------------------------------------------------------

def main():
    os.makedirs(OUT, exist_ok=True)
    check_against_module()
    print(f"Modelo V6: L1..L6 = {K.L1}, {K.L2}, {K.L3}, {K.L4}, {K.L5}, {K.L6} mm")
    hang = G(HANG)
    print(f"Pie con la pierna colgando (t = 0): X={hang[0]:.2f}  Y={hang[1]:.2f}  Z={hang[2]:.2f}")
    print("Límites control [°]:", LIM_CTRL.tolist())

    hs = HANG[2] + np.arange(5, 200, 5.0)
    rows = [analyze(h, LIM_CTRL) for h in hs]
    rows_urdf = [analyze(h, LIM_URDF) for h in hs]

    def papel(rw):
        """(X del papel, Y centro, Z centro) en el sistema del equipo."""
        return G([rw["xc"] if rw["side_safe"] else HANG[0],
                  rw["yc"] if rw["side_safe"] else HANG[1], rw["h"]])

    def centro_txt(rw):
        c = papel(rw)
        return f"({c[1]:.0f}, {c[2]:.0f})" if rw["side_safe"] else "—"

    print("\n| Sobre el pie | X papel | área | cuadrado | + lápiz | seguro | centro (Y, Z) | 1/κ papel mín | 1/κ 3D mín | rodilla mín | roll usado | margen |")
    for rw in rows:
        if rw["side_nolift"] == 0:
            continue
        print(f"| {rw['above']:.0f} | {papel(rw)[0]:.1f} | {rw['area']:.0f} cm² | {rw['side_nolift']:.0f} | "
              f"{rw['side']:.0f} | {rw['side_safe']:.0f} | {centro_txt(rw)} | "
              f"{rw['icp_min']:.2f} | {rw['ic_min']:.3f} | {rw['knee_min']:.1f}° | "
              f"{rw['t1'][0]:.1f}…{rw['t1'][1]:.1f}° | {rw['margin']:.1f}° |")

    best = max(rows, key=lambda rw: rw["side_safe"])
    best_u = max(rows_urdf, key=lambda rw: rw["side_safe"])

    print("\nCuadrado seguro [mm] y centro (Y, Z) según altura y elevación del lápiz:")
    lifts = (10, 20, 30, 40)
    print("| Sobre el pie | " + " | ".join(f"lápiz {l} mm" for l in lifts) + " |")
    for above in (best["above"] + d for d in (-25, -15, -5, 0, 5, 15)):
        cells = []
        for l in lifts:
            rw = analyze(HANG[2] + above, LIM_CTRL, l)
            cells.append(f"{rw['side_safe']:.0f}" + (f" {centro_txt(rw)}" if rw["side_safe"] else ""))
        print(f"| {above:.0f} | " + " | ".join(cells) + " |")

    c = papel(best)
    print(f"\nRECOMENDADO (lápiz {LIFT:g} mm): papel {best['above']:.0f} mm sobre el pie colgando "
          f"(plano X = {c[0]:.2f}), cuadrado seguro {best['side_safe']:.0f} mm, centro "
          f"Y = {c[1]:.2f}, Z = {c[2]:.2f} ({abs(c[2] - hang[2]):.1f} mm hacia fuera)")
    cu = papel(best_u)
    print(f"Con roll ≥ -20° (URDF V6): {best_u['above']:.0f} mm, {best_u['side_safe']:.0f} mm, "
          f"centro (Y, Z) = ({cu[1]:.1f}, {cu[2]:.1f})")

    t, ok, _ = K.ik_geometrico(*c, side=SIDE)
    if ok:
        J = R_G @ jac(*[np.array(v) for v in t]) * 180 / math.pi        # mm/rad, sistema del equipo
        U, S, _ = np.linalg.svd(J)
        print("Centro: t =", np.round(t, 1), "| valores singulares [mm/rad]", S.round(0),
              "| dirección débil (X,Y,Z)", U[:, -1].round(2))
        t2, ok2, _ = K.ik_geometrico(c[0] - LIFT, c[1], c[2], side=SIDE)
        print("Centro con lápiz arriba: t =", np.round(t2, 1))
    print(f"Centro en el mundo de RViz: {np.round(K.punto_a_mundo(c, SIDE), 1)}")

    for rw in rows:
        if rw["above"] in (best["above"] - 40, best["above"], best["above"] + 40):
            fig_plane_map(rw)
    fig_workspace(best["h"])
    fig_planes(rows, rows_urdf, best["h"])
    print("Figuras en", OUT, ":", sorted(os.listdir(OUT)))


if __name__ == "__main__":
    main()
