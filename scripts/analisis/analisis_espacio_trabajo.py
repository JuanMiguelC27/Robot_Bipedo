#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Análisis del espacio de trabajo, singularidades y superficie horizontal
óptima de escritura para la pierna IZQUIERDA del robot bípedo (CAD V6).

Modelo cinemático: Denavit-Hartenberg con articulación fantasma (1A2) y
eslabón L6, el mismo de la cinemática inversa algebraica del proyecto
(robot_kinematics/kinem_invers_leg_algebraico_izq.py, de donde se importan
L1..L6). Coincide con el modelo 3D urdf_completo (Pata_Pacial2URDFV6).

        theta   d     a     alpha
  0A1    0     -L1    L2    -90°
  1A2    q1     0     0      90°   <- fantasma (cadera roll)
  2A3    0      L3    L4      0
  3A4    q2     0     L5    180°   <- cadera pitch
  4A5    q3     0     L6      0    <- rodilla

Sistema de coordenadas (base 0 del modelo): +X hacia ARRIBA, Y adelante /
atrás, Z lateral. Con la pierna colgando, el pie está en (-640.1, 0, -307.0).

Convenio de ángulos. La interfaz (teleop_node) y el control (control_node)
usan t = (roll, pitch, rodilla) con t = 0 = pierna colgando. La relación con
los ángulos q de este modelo se obtuvo ajustando el modelo contra el URDF V6
(que el control mueve con la rodilla invertida):

        q1 = 180° - t1      q2 = -t2      q3 = t3

Los límites que se aplican son los del control (LIM_CTRL, en t). Si cambian
en control_node.py / teleop_node.py, hay que actualizarlos aquí.

Uso:
    python3 analisis_espacio_trabajo.py             # solo guarda los PNG
    python3 analisis_espacio_trabajo.py --mostrar   # además abre las figuras interactivas
Genera las figuras en docs/img/espacio_trabajo/ e imprime las tablas que
recoge docs/ESPACIO_TRABAJO_Y_SUPERFICIE_ESCRITURA.md.
"""
import io
import math
import os
import sys
import contextlib

import numpy as np
import matplotlib
MOSTRAR = "--mostrar" in sys.argv
if not MOSTRAR:
    matplotlib.use("Agg")
import matplotlib.pyplot as plt

REPO = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
sys.path.insert(0, os.path.join(REPO, "ros2_ws", "src", "robot_kinematics"))

with contextlib.redirect_stdout(io.StringIO()):
    from robot_kinematics import kinem_invers_leg_algebraico_izq as ka   # noqa: E402

OUT = os.path.join(REPO, "docs", "img", "espacio_trabajo")
L1, L2, L3, L4, L5, L6 = ka.L1, ka.L2, ka.L3, ka.L4, ka.L5, ka.L6

# Límites [°] en el convenio de la interfaz t = (roll, pitch, rodilla):
# robot_control/control_node.py (joint_limits_lower/upper) y
# robot_teleop/teleop_node.py (lower_deg/upper_deg).
LIM_CTRL = np.array([(0.0, 90.0), (-90.0, 90.0), (-90.0, 90.0)])
# Límites del URDF V6 (pierna izquierda), en el mismo convenio.
LIM_URDF = np.array([(-20.0, 90.0), (-90.0, 90.0), (-90.0, 90.0)])

STEP = 5.0                        # resolución de las rejillas [mm]
LIFT = 20.0                       # elevación del lápiz entre figuras [mm]
Q3_SAFE = 10.0                    # distancia mínima a la singularidad rodilla recta [°]
MARGIN_SAFE = 5.0                 # margen mínimo a los límites articulares [°]


# ----------------------------------------------------------------------------
#  Cinemática (vectorizada)
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


def fk(q1, q2, q3):
    """Posición del pie [mm] para q [rad] (pierna izquierda)."""
    T = (_dh(0 * q1, -L1, L2, -np.pi / 2) @ _dh(q1, 0, 0, np.pi / 2) @ _dh(0 * q1, L3, L4, 0)
         @ _dh(q2, 0, L5, np.pi) @ _dh(q3, 0, L6, 0))
    return T[..., 0, 3], T[..., 1, 3], T[..., 2, 3]


def ik_all(x, y, z):
    """Las 4 soluciones de la cinemática inversa izquierda (las dos raíces
    de r y los dos codos). La de r > 0 y codo 'arriba' es la de
    cinematica_inversa_pata_alg izquierda. Devuelve lista de (q1, q2, q3, ok)."""
    xp, zp = x - L2, z + L1
    disc = xp**2 + zp**2 - L3**2
    sols = []
    for sr in (1, -1):
        r = sr * np.sqrt(np.maximum(disc, 0))
        q1 = np.arctan2(L3, r) - np.arctan2(zp, xp)
        u, v = r - L4, y
        c3 = (u**2 + v**2 - L5**2 - L6**2) / (2 * L5 * L6)
        ok = (disc >= 0) & (np.abs(c3) <= 1 + 1e-9)
        c3 = np.clip(c3, -1, 1)
        for codo in (1, -1):
            q3 = np.arctan2(codo * np.sqrt(1 - c3**2), c3)
            A, B = L5 + L6 * np.cos(q3), L6 * np.sin(q3)
            det = A**2 + B**2
            q2 = np.arctan2((B * u + A * v) / det, (A * u - B * v) / det)
            sols.append((q1, q2, q3, ok))
    return sols


def q_to_t(q1, q2, q3):
    """Ángulos del modelo [rad] -> convenio de la interfaz [°], en (-180, 180]."""
    w = lambda a: (a + 180.0) % 360.0 - 180.0
    return w(180.0 - np.degrees(q1)), w(-np.degrees(q2)), w(np.degrees(q3))


def t_to_q(t1, t2, t3):
    return np.radians(180.0 - t1), np.radians(-t2), np.radians(t3)


def jac(q1, q2, q3, h=1e-6):
    """Jacobiano numérico 3x3 (filas X, Y, Z; columnas q1, q2, q3)."""
    cols = []
    for k in range(3):
        qp = [q1, q2, q3]; qm = [q1, q2, q3]
        qp[k] = qp[k] + h; qm[k] = qm[k] - h
        cols.append((np.stack(fk(*qp), -1) - np.stack(fk(*qm), -1)) / (2 * h))
    return np.stack(cols, -1)


def inv_cond(q1, q2, q3, rows=(0, 1, 2)):
    """1/κ del Jacobiano (o de sus filas): 1 = isotrópico, 0 = singular.
    rows=(1, 2) mide solo el movimiento dentro del plano horizontal (Y, Z)."""
    s = np.linalg.svd(jac(q1, q2, q3)[..., list(rows), :], compute_uv=False)
    return s[..., -1] / s[..., 0]


def check_against_module():
    """La FK vectorizada y la IK coinciden con el módulo del proyecto y con
    la cinemática directa de referencia."""
    rng = np.random.default_rng(0)
    for _ in range(300):
        q = rng.uniform([-1.0, -1.2, -1.2], [1.0, 1.2, 1.2])
        p = np.array(fk(*q))
        best = min(np.linalg.norm(np.array(fk(*s[:3])) - p) for s in ik_all(*p) if s[3])
        assert best < 1e-6
    # La IK izquierda es la del módulo con z -> -z respecto a la derecha:
    # comprobamos además que la raíz principal coincide con la versión
    # izquierda (z + L1) de cinematica_inversa_pata_alg.
    p = np.array(fk(*t_to_q(20.0, 15.0, 40.0)))
    q1, q2, q3, _ = ik_all(*p)[0]
    assert np.linalg.norm(np.array(fk(q1, q2, q3)) - p) < 1e-6


HANG = np.array(fk(*t_to_q(0.0, 0.0, 0.0)))     # pie con la pierna colgando


# ----------------------------------------------------------------------------
#  Plano horizontal X = h
# ----------------------------------------------------------------------------

YS = np.arange(-450, 450 + STEP, STEP)
ZS = HANG[2] + np.arange(-450, 450 + STEP, STEP)
GY, GZ = np.meshgrid(YS, ZS)
_cache = {}


def margin_deg(t, lim):
    return np.minimum.reduce([m for v, (lo, hi) in zip(t, lim) for m in (v - lo, hi - v)])


def plane(h, lim):
    """Rejilla del plano X = h. Un punto vale si alguna solución de la IK
    respeta los límites; se queda con la de mayor margen."""
    key = (float(h), lim.tobytes())
    if key in _cache:
        return _cache[key]
    X = np.full_like(GY, h)
    best_m = np.full(GY.shape, -np.inf)
    best_q = [np.full(GY.shape, np.nan) for _ in range(3)]
    for q1, q2, q3, ok in ik_all(X, GY, GZ):
        t = q_to_t(q1, q2, q3)
        m = np.where(ok, margin_deg(t, lim), -np.inf)
        better = m > best_m
        best_m = np.where(better, m, best_m)
        for k, qk in enumerate((q1, q2, q3)):
            best_q[k] = np.where(better, qk, best_q[k])
    good = best_m >= 0
    q = [np.where(good, qk, 0.0) for qk in best_q]
    knee = np.abs(np.degrees(q[2]))
    safe = good & (knee >= Q3_SAFE) & (best_m >= MARGIN_SAFE)
    _cache[key] = dict(good=good, safe=safe, q=q, t=q_to_t(*q),
                       icp=np.where(good, inv_cond(*q, rows=(1, 2)), np.nan),
                       ic=np.where(good, inv_cond(*q), np.nan),
                       margin=np.where(good, best_m, np.nan), knee=np.where(good, knee, np.nan))
    return _cache[key]


def largest_square(mask):
    """Mayor cuadrado (alineado con Y, Z) sin huecos en 'mask'.
    Devuelve (lado [mm], Y centro, Z centro, máscara). Entre cuadrados
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
        yc, zc = (YS[j] + YS[j - k + 1]) / 2, (ZS[i] + ZS[i - k + 1]) / 2
        d = math.hypot(yc, zc - HANG[2])
        if best is None or d < best[0]:
            best = (d, i, j, yc, zc)
    _, i, j, yc, zc = best
    sq = np.zeros_like(mask)
    sq[i - k + 1:i + 1, j - k + 1:j + 1] = True
    return (k - 1) * STEP, yc, zc, sq


def analyze(h, lim, lift=LIFT):
    p, pl = plane(h, lim), plane(h + lift, lim)
    side0 = largest_square(p["good"])[0]
    side = largest_square(p["good"] & pl["good"])[0]
    side_s, yc, zc, sq = largest_square(p["safe"] & pl["safe"])
    if not sq.any():
        sq = largest_square(p["good"] & pl["good"])[3]
    f = (lambda a, fn: fn(a[sq]) if sq.any() else np.nan)
    return dict(h=h, above=h - HANG[0], area=p["good"].sum() * STEP**2 / 100,
                side_nolift=side0, side=side, side_safe=side_s, yc=yc, zc=zc,
                icp_min=f(p["icp"], np.nanmin), ic_min=f(p["ic"], np.nanmin),
                margin=f(p["margin"], np.nanmin), knee_min=f(p["knee"], np.nanmin),
                t1=(f(p["t"][0], np.nanmin), f(p["t"][0], np.nanmax)), sq=sq)


# ----------------------------------------------------------------------------
#  Figuras
# ----------------------------------------------------------------------------

def fig_workspace(h_rec):
    """Cortes del espacio de trabajo con los límites del control."""
    n = 400
    fig, ax = plt.subplots(1, 2, figsize=(12, 5.8))
    # sagital: roll en 0, pitch y rodilla libres
    t2, t3 = np.meshgrid(np.linspace(*LIM_CTRL[1], n), np.linspace(*LIM_CTRL[2], n))
    q = t_to_q(np.zeros_like(t2), t2, t3)
    x, y, _ = fk(*q)
    det = np.abs(np.linalg.det(jac(*q)))
    sc = ax[0].scatter(y, x, c=det / det.max(), s=1, cmap="viridis", rasterized=True)
    sing = np.abs(t3) < 0.6
    ax[0].scatter(y[sing], x[sing], s=2, c="red", label="singularidad: rodilla recta")
    ax[0].plot(0, 0, "k^", ms=8); ax[0].text(15, 0, "base del modelo (cadera)", fontsize=8, va="center")
    # frontal: pitch y rodilla libres, roll en todo su rango
    zs, xs = [], []
    g2, g3 = np.meshgrid(np.linspace(*LIM_CTRL[1], 120), np.linspace(*LIM_CTRL[2], 120))
    for t1 in np.linspace(*LIM_CTRL[0], 400):
        q = t_to_q(np.full_like(g2, t1), g2, g3)
        x2, _, z2 = fk(*q); zs.append(z2.ravel()); xs.append(x2.ravel())
    ax[1].scatter(np.concatenate(zs), np.concatenate(xs), s=0.3, c="#1f6fd1", alpha=0.2, rasterized=True)
    ax[1].plot(0, 0, "k^", ms=8)
    for a in ax:
        a.axhline(h_rec, color="k", lw=1); a.axhline(h_rec + LIFT, color="k", ls="--", lw=1)
        a.plot(HANG[1] if a is ax[0] else HANG[2], HANG[0], "o", mfc="none", mec="k", ms=8)
        a.set_aspect("equal")
    ax[0].text(ax[0].get_xlim()[1], h_rec - 8, f"plano X = {h_rec:.0f}", fontsize=8, va="top", ha="right")
    ax[0].set(xlabel="Y [mm]  (adelante / atrás)", ylabel="X [mm]  (+ = arriba)",
              title="Corte sagital (roll = 0) con los límites del control")
    ax[0].legend(loc="upper right", fontsize=8)
    fig.colorbar(sc, ax=ax[0], label="|det J| normalizado  (0 = singular)", shrink=0.8)
    ax[1].set(xlabel="Z [mm]  (lateral)", ylabel="X [mm]",
              title=f"Corte frontal (roll {LIM_CTRL[0][0]:g}°…{LIM_CTRL[0][1]:g}°)")
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "espacio_trabajo_cortes.png"), dpi=110)
    if not MOSTRAR:
        plt.close(fig)


def fig_planes(rows, rows_urdf, h_rec):
    a = [r["above"] for r in rows]
    fig, ax = plt.subplots(1, 2, figsize=(12, 4.4))
    ax[0].plot(a, [r["side_safe"] for r in rows_urdf], ":", color="#999",
               label="seguro, con roll ≥ -20° (límite del URDF)")
    ax[0].plot(a, [r["side_nolift"] for r in rows], "o-", ms=4, label="solo el plano")
    ax[0].plot(a, [r["side"] for r in rows], "s-", ms=4, label=f"+ lápiz levantado {LIFT:g} mm")
    ax[0].plot(a, [r["side_safe"] for r in rows], "^-", ms=4,
               label=f"seguro: rodilla ≥ {Q3_SAFE:g}°, margen ≥ {MARGIN_SAFE:g}°")
    ax[0].axvline(h_rec - HANG[0], color="k", lw=1)
    ax[0].set(xlabel="Altura del plano sobre el pie colgando [mm]", ylabel="Lado del mayor cuadrado [mm]",
              title="Área de escritura según la altura del plano")
    ax[0].legend(fontsize=7.5, loc="lower right"); ax[0].grid(alpha=0.3)
    ax[1].plot(a, [r["icp_min"] for r in rows], "o-", ms=4, label="1/κ en el plano (Y, Z): mínimo")
    ax[1].plot(a, [r["ic_min"] for r in rows], "s-", ms=4, label="1/κ en 3D (incluye subir/bajar): mínimo")
    ax[1].set_ylim(0, 1.05); ax[1].axvline(h_rec - HANG[0], color="k", lw=1)
    ax[1].set(xlabel="Altura del plano sobre el pie colgando [mm]", ylabel="1/κ(J)  (1 = ideal)",
              title="Calidad del movimiento en el cuadrado seguro")
    ax[1].legend(fontsize=8, loc="lower left"); ax[1].grid(alpha=0.3)
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "comparacion_planos.png"), dpi=110)
    if not MOSTRAR:
        plt.close(fig)


def fig_plane_map(rw):
    h = rw["h"]
    p = plane(h, LIM_CTRL)
    fig, ax = plt.subplots(1, 2, figsize=(12, 5.4))
    ext = (YS[0] - STEP / 2, YS[-1] + STEP / 2, ZS[0] - STEP / 2, ZS[-1] + STEP / 2)
    im = ax[0].imshow(p["icp"], origin="lower", extent=ext, cmap="viridis", vmin=0, vmax=1)
    fig.colorbar(im, ax=ax[0], label="1/κ del movimiento en el plano (Y, Z)", shrink=0.85)
    im2 = ax[1].imshow(p["margin"], origin="lower", extent=ext, cmap="magma", vmin=0, vmax=60)
    fig.colorbar(im2, ax=ax[1], label="margen al límite articular más cercano [°]", shrink=0.85)
    lost = plane(h, LIM_URDF)["good"] & ~p["good"]
    for a, t in zip(ax, ("Manipulabilidad en el plano", "Margen a los límites")):
        if lost.any():
            a.contourf(GY, GZ, lost.astype(float), levels=[0.5, 1.5], colors="none", hatches=["////"])
        a.plot(0, HANG[2], "k+", ms=12)
        if rw["side_safe"] > 0:
            s = rw["side_safe"] / 2
            a.add_patch(plt.Rectangle((rw["yc"] - s, rw["zc"] - s), 2 * s, 2 * s,
                                      fill=False, ec="#2ca02c", lw=2.5))
            a.text(rw["yc"] - s, rw["zc"] + s + 8,
                   f"seguro {rw['side_safe']:.0f} × {rw['side_safe']:.0f} mm", fontsize=8,
                   color="#1b7a1b", bbox=dict(fc="white", ec="none", alpha=0.85, pad=1.5))
        a.set(xlabel="Y [mm]  (adelante / atrás)", ylabel="Z [mm]  (lateral)",
              title=f"{t}\nplano a {rw['above']:.0f} mm sobre el pie colgando")
        a.set_xlim(-400, 400); a.set_ylim(HANG[2] - 400, HANG[2] + 400); a.set_aspect("equal")
    ax[0].text(-390, HANG[2] - 390, f"+ : vertical del pie colgando.  Rayado: lo recorta el control (roll < {LIM_CTRL[0][0]:g}°)",
               fontsize=7.5, va="bottom")
    fig.tight_layout(); fig.savefig(os.path.join(OUT, f"plano_{rw['above']:.0f}mm.png"), dpi=110)
    if not MOSTRAR:
        plt.close(fig)


# ----------------------------------------------------------------------------

def main():
    os.makedirs(OUT, exist_ok=True)
    for f in os.listdir(OUT):
        if f.endswith(".png"):
            os.remove(os.path.join(OUT, f))
    check_against_module()
    print(f"Modelo V6: L1..L6 = {L1}, {L2}, {L3}, {L4}, {L5}, {L6} mm")
    print(f"Pie con la pierna colgando (t = 0): X={HANG[0]:.1f}  Y={HANG[1]:.1f}  Z={HANG[2]:.1f}")
    print("Límites control [°]:", LIM_CTRL.tolist(), " URDF:", LIM_URDF.tolist())

    hs = HANG[0] + np.arange(5, 200, 5.0)
    rows = [analyze(h, LIM_CTRL) for h in hs]
    rows_urdf = [analyze(h, LIM_URDF) for h in hs]

    print("\n| Sobre el pie | X plano | área | cuadrado | + lápiz | seguro | centro (Y, Z) | 1/κ plano mín | 1/κ 3D mín | rodilla mín | roll usado | margen |")
    for rw in rows:
        if rw["side_nolift"] == 0:
            continue
        print(f"| {rw['above']:.0f} | {rw['h']:.1f} | {rw['area']:.0f} cm² | {rw['side_nolift']:.0f} | "
              f"{rw['side']:.0f} | {rw['side_safe']:.0f} | ({rw['yc']:.0f}, {rw['zc']:.0f}) | "
              f"{rw['icp_min']:.2f} | {rw['ic_min']:.3f} | {rw['knee_min']:.1f}° | "
              f"{rw['t1'][0]:.1f}…{rw['t1'][1]:.1f}° | {rw['margin']:.1f}° |")

    print("\nCuadrado seguro [mm] y centro (Y, Z) según altura y elevación del lápiz:")
    lifts = (10, 20, 30, 40)
    print("| Sobre el pie | " + " | ".join(f"lápiz {l} mm" for l in lifts) + " |")
    for above in range(10, 200, 10):
        cells = []
        for l in lifts:
            rw = analyze(HANG[0] + above, LIM_CTRL, l)
            cells.append(f"{rw['side_safe']:.0f}" + (f" ({rw['yc']:.0f}, {rw['zc']:.0f})" if rw["side_safe"] else ""))
        print(f"| {above} | " + " | ".join(cells) + " |")

    best = max(rows, key=lambda rw: rw["side_safe"])
    best_u = max(rows_urdf, key=lambda rw: rw["side_safe"])
    print(f"\nRECOMENDADO (lápiz {LIFT:g} mm): plano {best['above']:.0f} mm sobre el pie colgando "
          f"(X = {best['h']:.1f}), cuadrado seguro {best['side_safe']:.0f} mm, centro "
          f"Y = {best['yc']:.1f}, Z = {best['zc']:.1f} ({best['zc'] - HANG[2]:+.1f} mm lateral)")
    print(f"Con roll ≥ -20° (URDF): {best_u['above']:.0f} mm, {best_u['side_safe']:.0f} mm, "
          f"centro ({best_u['yc']:.1f}, {best_u['zc']:.1f})")

    # Detalle en el centro del cuadrado recomendado
    q = None
    for s in ik_all(np.array(best["h"]), np.array(best["yc"]), np.array(best["zc"])):
        t = q_to_t(*s[:3])
        if s[3] and margin_deg(t, LIM_CTRL) >= 0:
            q = s[:3]; break
    if q is not None:
        J = jac(*[np.array(v) for v in q]); U, S, _ = np.linalg.svd(J)
        print("Centro: t =", np.round(q_to_t(*q), 1), "| valores singulares", S.round(0),
              "| dirección débil (X,Y,Z)", U[:, -1].round(2))
        for s in ik_all(np.array(best["h"] + LIFT), np.array(best["yc"]), np.array(best["zc"])):
            t = q_to_t(*s[:3])
            if s[3] and margin_deg(t, LIM_CTRL) >= 0:
                print("Centro con lápiz arriba: t =", np.round(t, 1)); break

    for rw in rows:
        if rw["above"] in (best["above"] - 40, best["above"], best["above"] + 40):
            fig_plane_map(rw)
    fig_workspace(best["h"])
    fig_planes(rows, rows_urdf, best["h"])
    print("Figuras en", OUT, ":", sorted(os.listdir(OUT)))
    if MOSTRAR:
        plt.show()


if __name__ == "__main__":
    main()
