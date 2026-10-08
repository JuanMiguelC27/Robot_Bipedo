#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Trazador: dibujo a mano alzada -> trayectoria de vectores (.txt)

Dibuja con el ratón / lápiz sobre un plano de trabajo (255 x 255 mm por defecto: el área segura de la pierna).
El programa identifica la figura (triángulo, cuadrado, polígono, círculo, línea
o trazo libre), interpola los puntos con un paso fijo y exporta la lista de
vectores [x,y,z] en mm, lista para la cinemática inversa de la pierna.
Las figuras se dibujan a Z constante (200 mm por defecto); entre figuras
distintas el lápiz se levanta (Z + 100 mm), se desplaza y vuelve a bajar.

Uso:        python3 trazador.py
Requisitos: Python 3 + Tkinter   (sudo apt install python3-tk)

Sistema de coordenadas: origen (0,0) en la esquina INFERIOR IZQUIERDA del
plano, X hacia la derecha, Y hacia arriba, unidades en mm.
Z es configurable en la interfaz (plano de dibujo y elevación del lápiz).

"Exportar para la pierna" guarda la trayectoria ya convertida al sistema de
coordenadas de la pierna del robot (el de las pestañas de cinemática
inversa de robot_teleop: X a lo largo de la pierna hacia abajo, Y adelante,
Z lateral; el papel es un plano X = constante), en formato "x, y, z", listo
para la pestaña Trayectorias o "Abrir archivo..." de la interfaz. Usa
robot_kinematics/kinem_v6.py para marcar en rojo los puntos que la pierna no
alcanza dentro de los límites del control.
"""
import math
import os
import re
import sys
import time
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

CANVAS_MAX_PX = 600   # tamaño máximo del área de dibujo en pantalla
REPO = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
KINEMATICS_PKG = os.path.join(REPO, "ros2_ws", "src", "robot_kinematics")
LEG_TRAJ_DIR = os.path.join(KINEMATICS_PKG, "trayectorias")
PAD = 34              # margen para las etiquetas de los ejes
GRID_MM = 10          # separación de la cuadrícula


# =============================================================================
#  Geometría (independiente de la interfaz)
# =============================================================================

def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def line_dist(p, a, b):
    """Distancia de p a la recta que pasa por a y b."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    L = math.hypot(dx, dy)
    if L == 0:
        return dist(p, a)
    return abs(dx * (a[1] - p[1]) - dy * (a[0] - p[0])) / L


def seg_dist(p, a, b):
    """Distancia de p al segmento ab."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    L2 = dx * dx + dy * dy
    if L2 == 0:
        return dist(p, a)
    t = max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / L2))
    return dist(p, (a[0] + t * dx, a[1] + t * dy))


def path_length(pts):
    return sum(dist(a, b) for a, b in zip(pts, pts[1:]))


def resample(pts, spacing):
    """Remuestrea una polilínea con puntos equiespaciados (el ratón no
    entrega eventos a intervalos regulares)."""
    if len(pts) < 2:
        return list(pts)
    out = [pts[0]]
    acc = 0.0
    prev = pts[0]
    for p in pts[1:]:
        d = dist(prev, p)
        while acc + d >= spacing and d > 0:
            t = (spacing - acc) / d
            q = (prev[0] + t * (p[0] - prev[0]), prev[1] + t * (p[1] - prev[1]))
            out.append(q)
            prev, acc = q, 0.0
            d = dist(prev, p)
        acc += d
        prev = p
    if dist(out[-1], pts[-1]) > 1e-9:
        out.append(pts[-1])
    return out


def rdp(pts, eps):
    """Simplificación Ramer-Douglas-Peucker (iterativa)."""
    if len(pts) < 3:
        return list(pts)
    keep = [False] * len(pts)
    keep[0] = keep[-1] = True
    stack = [(0, len(pts) - 1)]
    while stack:
        i, j = stack.pop()
        dmax, idx = 0.0, -1
        for k in range(i + 1, j):
            d = line_dist(pts[k], pts[i], pts[j])
            if d > dmax:
                dmax, idx = d, k
        if dmax > eps:
            keep[idx] = True
            stack += [(i, idx), (idx, j)]
    return [p for p, k in zip(pts, keep) if k]


def prune_closed(verts, eps):
    """Elimina vértices casi colineales o lados muy cortos de un polígono
    cerrado (p. ej. el 'ganchito' que queda al cerrar la figura a mano)."""
    v = list(verts)
    changed = True
    while changed and len(v) > 3:
        changed = False
        for i in range(len(v)):
            a, b, c = v[i - 1], v[i], v[(i + 1) % len(v)]
            if line_dist(b, a, c) < eps or dist(a, b) < eps:
                del v[i]
                changed = True
                break
    return v


def signed_area(v):
    return 0.5 * sum(v[i - 1][0] * v[i][1] - v[i][0] * v[i - 1][1] for i in range(len(v)))


def polygon_name(v):
    n = len(v)
    names = {3: "triángulo", 5: "pentágono", 6: "hexágono",
             7: "heptágono", 8: "octágono"}
    if n == 4:
        ang_ok = True
        for i in range(4):
            a, b, c = v[i - 1], v[i], v[(i + 1) % 4]
            u = (a[0] - b[0], a[1] - b[1])
            w = (c[0] - b[0], c[1] - b[1])
            cosang = (u[0] * w[0] + u[1] * w[1]) / (math.hypot(*u) * math.hypot(*w) + 1e-12)
            if abs(math.degrees(math.acos(max(-1, min(1, cosang)))) - 90) > 15:
                ang_ok = False
        if ang_ok:
            sides = [dist(v[i - 1], v[i]) for i in range(4)]
            return "cuadrado" if max(sides) / min(sides) < 1.2 else "rectángulo"
        return "cuadrilátero"
    return names.get(n, f"polígono de {n} lados")


def analyze_stroke(raw, mode="figura", sens_pct=6.0, close_auto=True, step=1.0):
    """Analiza un trazo (lista de puntos en mm).
    Devuelve dict(name, verts, closed) o None si el trazo es despreciable."""
    pts = resample(raw, 0.5)
    if len(pts) < 2:
        return None
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    diag = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
    if diag < 1.0:
        return None

    closed = (close_auto and len(pts) > 4
              and dist(pts[0], pts[-1]) < max(0.15 * diag, 2.0)
              and path_length(pts) > 1.8 * diag)

    # ---- Trazo libre: conserva la forma, solo quita ruido ----
    if mode == "libre":
        if closed:
            v = rdp(pts + [pts[0]], 0.3)[:-1]
            return {"name": "trazo libre cerrado", "verts": v, "closed": True}
        return {"name": "trazo libre", "verts": rdp(pts, 0.3), "closed": False}

    # ---- Detección de figura ----
    eps = sens_pct / 100.0 * diag
    if not closed:
        v = rdp(pts, eps)
        name = "línea" if len(v) == 2 else f"polilínea ({len(v) - 1} segmentos)"
        return {"name": name, "verts": v, "closed": False}

    v = prune_closed(rdp(pts + [pts[0]], eps)[:-1], eps)

    # ¿Círculo o polígono? Se compara el error de ajuste de ambos modelos.
    cx = sum(xs) / len(xs)
    cy = sum(ys) / len(ys)
    radii = [math.hypot(p[0] - cx, p[1] - cy) for p in pts]
    r = sum(radii) / len(radii)
    circ_err = sum(abs(ri - r) for ri in radii) / len(radii)
    if len(v) >= 3:
        poly_err = sum(min(seg_dist(p, v[i - 1], v[i]) for i in range(len(v)))
                       for p in pts) / len(pts)
    else:
        poly_err = float("inf")

    if circ_err < poly_err and circ_err < 0.12 * r:
        # Círculo ideal, empezando donde empezó el trazo y en el mismo sentido
        a0 = math.atan2(pts[0][1] - cy, pts[0][0] - cx)
        sense = 1 if signed_area(pts) >= 0 else -1
        n = max(24, math.ceil(2 * math.pi * r / max(step, 0.1)))
        v = [(cx + r * math.cos(a0 + sense * 2 * math.pi * i / n),
              cy + r * math.sin(a0 + sense * 2 * math.pi * i / n)) for i in range(n)]
        return {"name": f"círculo (r≈{r:.1f} mm)", "verts": v, "closed": True}

    if len(v) < 3:
        v = rdp(pts + [pts[0]], 0.3)[:-1]
        return {"name": "trazo libre cerrado", "verts": v, "closed": True}
    return {"name": polygon_name(v), "verts": v, "closed": True}


def rnd(x, dec):
    """Redondeo 'mitad hacia arriba' (evita el redondeo bancario de round())."""
    f = 10 ** dec
    val = math.floor(x * f + 0.5) / f
    return int(val) if dec == 0 else val


def interpolate(verts, step=1.0, dec=0):
    """Interpola linealmente entre vértices consecutivos (x,y) o (x,y,z).
    El número de pasos de cada segmento es max(|dx|,|dy|,|dz|)/paso, de modo
    que (0,0,200)->(10,10,200) da [0,0,200],[1,1,200],...,[10,10,200]."""
    out = []

    def add(p):
        q = tuple(rnd(c, dec) for c in p)
        if not out or out[-1] != q:
            out.append(q)

    if not verts:
        return out
    add(verts[0])
    for a, b in zip(verts, verts[1:]):
        d = [cb - ca for ca, cb in zip(a, b)]
        n = max(1, math.ceil(max(abs(c) for c in d) / step - 1e-9))
        for i in range(1, n + 1):
            add(tuple(ca + dc * i / n for ca, dc in zip(a, d)))
    return out


def build_path(figs, W, H, step=1.0, dec=0, normalize=False, z_draw=200.0, lift=100.0):
    """Construye la trayectoria 3D (x,y,z) de todas las figuras.
    Cada figura se dibuja en el plano z = z_draw (cerrando las figuras
    cerradas). Entre una figura y la siguiente se levanta el lápiz:
    sube 'lift' mm en Z, se desplaza en XY hasta el inicio de la siguiente
    figura y vuelve a bajar. Todo el recorrido se interpola con el mismo paso."""
    polys = []
    for f in figs:
        vs = list(f["verts"])
        if f["closed"]:
            vs.append(vs[0])
        polys.append([(min(max(x, 0.0), W), min(max(y, 0.0), H)) for x, y in vs])
    if not polys:
        return []
    if normalize:
        mx = min(x for vs in polys for x, _ in vs)
        my = min(y for vs in polys for _, y in vs)
        polys = [[(x - mx, y - my) for x, y in vs] for vs in polys]
    zd = rnd(z_draw, dec)
    zu = rnd(z_draw + lift, dec)

    verts = []
    for k, vs in enumerate(polys):
        vs = [(rnd(x, dec), rnd(y, dec)) for x, y in vs]
        if k > 0:
            px, py, _ = verts[-1]
            verts.append((px, py, zu))              # sube el lápiz
            verts.append((vs[0][0], vs[0][1], zu))  # se desplaza en el aire
        verts.extend((x, y, zd) for x, y in vs)     # baja y dibuja
    return interpolate(verts, step, dec)


def count_lifts(path):
    """Número de veces que se levanta el lápiz (la Z del dibujo es la del
    primer punto)."""
    if not path:
        return 0
    zd = path[0][2]
    return sum(1 for a, b in zip(path, path[1:]) if a[2] == zd and b[2] != zd)


def format_path(path, fmt="lineas", dec=0):
    def num(v):
        return str(v) if dec == 0 else f"{v:.{dec}f}"
    if fmt == "csv":
        return "".join(",".join(num(c) for c in p) + "\n" for p in path)
    items = ["[" + ",".join(num(c) for c in p) + "]" for p in path]
    if fmt == "una_linea":
        return ", ".join(items) + "\n"
    return "\n".join(items) + "\n"


def parse_path(text, z_default=200.0):
    """Lee [x,y,z] / [x,y] o 'x,y,z' / 'x,y'. A los puntos 2D se les asigna
    z_default."""
    num = r"(-?\d+(?:\.\d+)?)"
    sep = r"\s*[,;\s]\s*"
    rows = re.findall(r"\[\s*" + num + r"\s*,\s*" + num + r"\s*(?:,\s*" + num + r"\s*)?\]", text)
    if not rows:
        rows = re.findall(r"^\s*" + num + sep + num + r"(?:" + sep + num + r")?\s*$", text, re.M)
    return [(float(x), float(y), float(z) if z else z_default) for x, y, z in rows]


# =============================================================================
#  Marco de la pierna (sistema de las pestañas de cinemática inversa)
# =============================================================================
#
# X a lo largo de la pierna hacia ABAJO, Y hacia adelante, Z lateral (la
# pierna izquierda está en -Z, la derecha en +Z; +Z es el lado derecho del
# robot). El robot escribe sobre un papel HORIZONTAL: un plano X = cte.
#
# Papel:   u hacia la derecha, v hacia arriba, w = altura del lápiz en el
#          trazador. Se ve como lo vería alguien de pie DETRÁS del robot,
#          mirando hacia adelante y hacia abajo: arriba del dibujo =
#          adelante del robot, derecha del dibujo = derecha del robot.
#
#     X = x_papel - levantar * (w - w_dibujo) / (w_max - w_dibujo)
#     Y = yc + (v - alto/2) * escala
#     Z = zc + (u - ancho/2) * escala      (con espejo: -(u - ancho/2))
#
# Valores por defecto: los de kinem_v6.PAPEL_RECOMENDADO (papel 165 mm por
# encima del pie colgando, área segura de 255 x 255 mm, lápiz levantado
# 20 mm; límites del control con roll >= 0°).

PAPEL_RECOMENDADO = {
    "left": dict(x_paper=685.28, yc=0.0, zc=-489.48, lado=255.0),
    "right": dict(x_paper=685.28, yc=0.0, zc=489.48, lado=255.0),
}
LADOS = {"Izquierda": "left", "Derecha": "right"}


def paper_to_leg(path, x_paper=685.28, yc=0.0, zc=-489.48, width=255.0, height=255.0,
                 scale=1.0, mirror=False, lift=20.0):
    """Convierte una trayectoria (u, v, w) del papel al sistema de la pierna.
    La altura del dibujo (lápiz abajo) es la del primer punto; los puntos
    con w mayor se levantan proporcionalmente hasta 'lift' mm (X menor)."""
    if not path:
        return []
    w_draw = path[0][2]
    w_span = max(p[2] for p in path) - w_draw
    sign = -1 if mirror else 1
    return [(x_paper - (lift * (w - w_draw) / w_span if w_span > 0 else 0.0),
             yc + (v - height / 2) * scale,
             zc + sign * (u - width / 2) * scale)
            for u, v, w in path]


def format_leg_path(leg, header=""):
    lines = [f"# {h}\n" for h in header.splitlines()]
    lines += [f"{x:.2f}, {y:.2f}, {z:.2f}\n" for x, y, z in leg]
    return "".join(lines)


def load_leg_ik():
    """Importa robot_kinematics.kinem_v6. Devuelve el módulo, o None."""
    if KINEMATICS_PKG not in sys.path:
        sys.path.insert(0, KINEMATICS_PKG)
    try:
        from robot_kinematics import kinem_v6
    except Exception:
        return None
    return kinem_v6


def load_perfiles():
    """Importa robot_kinematics.perfiles_temporales (ley temporal del
    lápiz). Devuelve el módulo, o None."""
    if KINEMATICS_PKG not in sys.path:
        sys.path.insert(0, KINEMATICS_PKG)
    try:
        from robot_kinematics import perfiles_temporales
    except Exception:
        return None
    return perfiles_temporales


LEYES = ("lineal", "cubico", "quintico", "trapezoidal", "tiempo_minimo")


def unreachable_points(leg, ik, side="left"):
    """Índices de los puntos que la pierna no alcanza dentro de los límites
    del control (la misma comprobación que hace la interfaz)."""
    bad = []
    seed = None
    for i, (x, y, z) in enumerate(leg):
        t, ok, _ = ik.ik_geometrico(x, y, z, side=side, semilla_deg=seed)
        if ok:
            seed = t
        else:
            bad.append(i)
    return bad


# =============================================================================
#  Interfaz gráfica
# =============================================================================

FORMATOS = {
    "[x,y,z] uno por línea": "lineas",
    "[x,y,z], [x,y,z], ... en una línea": "una_linea",
    "x,y,z (CSV)": "csv",
}


class Trazador(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Trazador – dibujo a trayectoria")
        self.strokes = []      # trazos en mm
        self.current = None
        self.figs = []
        self.path = []
        self.loaded = False    # True si la trayectoria viene de un .txt
        self.sim_job = None
        self._proc_job = None
        self.bad = set()       # índices de puntos fuera de alcance
        self.ik = load_leg_ik()

        self.var_w = tk.DoubleVar(value=PAPEL_RECOMENDADO["left"]["lado"])
        self.var_h = tk.DoubleVar(value=PAPEL_RECOMENDADO["left"]["lado"])
        self.var_mode = tk.StringVar(value="Detectar figura")
        self.var_sens = tk.DoubleVar(value=6)
        self.var_step = tk.DoubleVar(value=1)
        self.var_dec = tk.IntVar(value=0)
        self.var_close = tk.BooleanVar(value=True)
        self.var_norm = tk.BooleanVar(value=False)
        self.var_fmt = tk.StringVar(value=list(FORMATOS)[0])
        self.var_show_pts = tk.BooleanVar(value=True)
        self.var_z = tk.DoubleVar(value=200)
        self.var_lift = tk.DoubleVar(value=100)
        self.var_side = tk.StringVar(value="Izquierda")
        rec = PAPEL_RECOMENDADO["left"]
        self.var_xp = tk.DoubleVar(value=rec["x_paper"])
        self.var_yc = tk.DoubleVar(value=rec["yc"])
        self.var_zc = tk.DoubleVar(value=rec["zc"])
        self.var_leg_lift = tk.DoubleVar(value=20)
        self.var_scale = tk.DoubleVar(value=1)
        self.var_mirror = tk.BooleanVar(value=False)
        self.var_check = tk.BooleanVar(value=self.ik is not None)
        self.reach = tk.StringVar(value="")
        self.perf = load_perfiles()
        self.var_ley = tk.StringVar(value=self.perf.NOMBRES["quintico"] if self.perf else "")
        self.var_v_lapiz = tk.DoubleVar(value=20)
        self.var_a_lapiz = tk.DoubleVar(value=100)
        self.var_esquinas = tk.BooleanVar(value=True)
        self.status = tk.StringVar(value="Dibuja con el botón izquierdo del ratón.")
        self.info = tk.StringVar(value="Sin trayectoria")

        self._build_ui()
        self._set_area()
        for v in (self.var_mode, self.var_sens, self.var_step, self.var_dec,
                  self.var_close, self.var_norm, self.var_show_pts, self.var_fmt,
                  self.var_z, self.var_lift, self.var_xp, self.var_yc, self.var_zc,
                  self.var_scale, self.var_mirror, self.var_check, self.var_leg_lift):
            v.trace_add("write", lambda *_: self._schedule_process())

    # ---------------------------------------------------------------- UI ----
    def _build_ui(self):
        main = ttk.Frame(self, padding=6)
        main.pack(fill="both", expand=True)

        self.canvas = tk.Canvas(main, bg="#f4f4f4", highlightthickness=0, cursor="pencil")
        self.canvas.grid(row=0, column=0, sticky="nw")
        self.canvas.bind("<ButtonPress-1>", self._on_press)
        self.canvas.bind("<B1-Motion>", self._on_drag)
        self.canvas.bind("<ButtonRelease-1>", self._on_release)
        self.canvas.bind("<Motion>", self._on_motion)

        side = ttk.Frame(main, padding=(10, 0))
        side.grid(row=0, column=1, sticky="ns")

        g = ttk.LabelFrame(side, text="Plano de trabajo (mm)", padding=6)
        g.pack(fill="x", pady=3)
        ttk.Label(g, text="Ancho").grid(row=0, column=0, sticky="w")
        ttk.Spinbox(g, from_=10, to=1000, increment=10, textvariable=self.var_w,
                    width=7).grid(row=0, column=1)
        ttk.Label(g, text="Alto").grid(row=1, column=0, sticky="w")
        ttk.Spinbox(g, from_=10, to=1000, increment=10, textvariable=self.var_h,
                    width=7).grid(row=1, column=1)
        ttk.Button(g, text="Aplicar", command=self._set_area).grid(row=0, column=2, rowspan=2, padx=6)

        p = ttk.LabelFrame(side, text="Procesamiento", padding=6)
        p.pack(fill="x", pady=3)
        ttk.Label(p, text="Modo").grid(row=0, column=0, sticky="w")
        ttk.Combobox(p, textvariable=self.var_mode, state="readonly", width=16,
                     values=["Detectar figura", "Trazo libre"]).grid(row=0, column=1, columnspan=2, sticky="w")
        ttk.Label(p, text="Tolerancia esquinas %").grid(row=1, column=0, sticky="w")
        ttk.Scale(p, from_=2, to=15, variable=self.var_sens, length=110).grid(row=1, column=1)
        self.lbl_sens = ttk.Label(p, width=4)
        self.lbl_sens.grid(row=1, column=2)
        self.var_sens.trace_add("write", lambda *_: self.lbl_sens.config(text=f"{self.var_sens.get():.0f}"))
        self.lbl_sens.config(text="6")
        ttk.Label(p, text="Paso interpolación (mm)").grid(row=2, column=0, sticky="w")
        ttk.Spinbox(p, from_=0.1, to=20, increment=0.5, textvariable=self.var_step,
                    width=7).grid(row=2, column=1, sticky="w")
        ttk.Label(p, text="Decimales").grid(row=3, column=0, sticky="w")
        ttk.Spinbox(p, from_=0, to=3, increment=1, textvariable=self.var_dec,
                    width=7).grid(row=3, column=1, sticky="w")
        ttk.Checkbutton(p, text="Cerrar figuras automáticamente",
                        variable=self.var_close).grid(row=4, column=0, columnspan=3, sticky="w")
        ttk.Checkbutton(p, text="Normalizar figura al origen (0,0)",
                        variable=self.var_norm).grid(row=5, column=0, columnspan=3, sticky="w")
        ttk.Checkbutton(p, text="Mostrar puntos interpolados",
                        variable=self.var_show_pts).grid(row=6, column=0, columnspan=3, sticky="w")

        z = ttk.LabelFrame(side, text="Eje Z (mm)", padding=6)
        z.pack(fill="x", pady=3)
        ttk.Label(z, text="Z del plano de dibujo").grid(row=0, column=0, sticky="w")
        ttk.Spinbox(z, from_=-1000, to=1000, increment=10, textvariable=self.var_z,
                    width=7).grid(row=0, column=1, sticky="w")
        ttk.Label(z, text="Elevación al levantar").grid(row=1, column=0, sticky="w")
        ttk.Spinbox(z, from_=-1000, to=1000, increment=10, textvariable=self.var_lift,
                    width=7).grid(row=1, column=1, sticky="w")
        ttk.Label(z, text="(negativa si en tu robot Z crece hacia abajo)",
                  foreground="#777").grid(row=2, column=0, columnspan=2, sticky="w")

        k = ttk.LabelFrame(side, text="Papel en la pierna (mm, sistema de las pestañas de IK)", padding=6)
        k.pack(fill="x", pady=3)
        ttk.Label(k, text="Pierna").grid(row=0, column=0, sticky="w")
        cb = ttk.Combobox(k, textvariable=self.var_side, state="readonly", width=10,
                          values=list(LADOS))
        cb.grid(row=0, column=1, sticky="w")
        cb.bind("<<ComboboxSelected>>", lambda e: self._set_side_defaults())
        ttk.Label(k, text="X del papel (hacia abajo)").grid(row=1, column=0, sticky="w")
        ttk.Spinbox(k, from_=-1500, to=1500, increment=5, textvariable=self.var_xp,
                    width=8).grid(row=1, column=1, sticky="w")
        ttk.Label(k, text="Centro Y (adelante)").grid(row=2, column=0, sticky="w")
        ttk.Spinbox(k, from_=-1500, to=1500, increment=5, textvariable=self.var_yc,
                    width=8).grid(row=2, column=1, sticky="w")
        ttk.Label(k, text="Centro Z (lateral)").grid(row=3, column=0, sticky="w")
        ttk.Spinbox(k, from_=-1500, to=1500, increment=5, textvariable=self.var_zc,
                    width=8).grid(row=3, column=1, sticky="w")
        ttk.Label(k, text="Levantar lápiz").grid(row=4, column=0, sticky="w")
        ttk.Spinbox(k, from_=0, to=200, increment=5, textvariable=self.var_leg_lift,
                    width=8).grid(row=4, column=1, sticky="w")
        ttk.Label(k, text="Escala").grid(row=5, column=0, sticky="w")
        ttk.Spinbox(k, from_=0.05, to=10, increment=0.05, textvariable=self.var_scale,
                    width=8).grid(row=5, column=1, sticky="w")
        ttk.Checkbutton(k, text="Espejo (izquierda/derecha)",
                        variable=self.var_mirror).grid(row=6, column=0, columnspan=2, sticky="w")
        ttk.Button(k, text="Valores recomendados",
                   command=self._set_side_defaults).grid(row=7, column=0, columnspan=2, sticky="w")
        chk = ttk.Checkbutton(k, text="Comprobar alcance de la pierna",
                              variable=self.var_check)
        chk.grid(row=8, column=0, columnspan=2, sticky="w")
        if self.ik is None:
            chk.state(["disabled"])
            self.reach.set("No se encontró robot_kinematics:\nsin comprobación de alcance.")
        self.lbl_reach = ttk.Label(k, textvariable=self.reach, wraplength=260, justify="left")
        self.lbl_reach.grid(row=9, column=0, columnspan=2, sticky="w")

        lt = ttk.LabelFrame(side, text="Ley temporal del lápiz", padding=6)
        lt.pack(fill="x", pady=3)
        if self.perf is None:
            ttk.Label(lt, text="No se encontró robot_kinematics:\nla simulación va a ritmo fijo.").grid(
                row=0, column=0, sticky="w")
        else:
            cb_ley = ttk.Combobox(lt, textvariable=self.var_ley, state="readonly", width=30,
                                  values=[self.perf.NOMBRES[m] for m in LEYES])
            cb_ley.grid(row=0, column=0, columnspan=4, sticky="w")
            cb_ley.bind("<<ComboboxSelected>>", lambda e: self._mostrar_condiciones())
            ttk.Label(lt, text="v máx [mm/s]").grid(row=1, column=0, sticky="w")
            self.sp_v = ttk.Spinbox(lt, from_=1, to=200, increment=5, textvariable=self.var_v_lapiz,
                                    width=6)
            self.sp_v.grid(row=1, column=1, sticky="w")
            ttk.Label(lt, text="a máx [mm/s²]").grid(row=1, column=2, sticky="w")
            self.sp_a = ttk.Spinbox(lt, from_=10, to=2000, increment=10, textvariable=self.var_a_lapiz,
                                    width=6)
            self.sp_a.grid(row=1, column=3, sticky="w")
            ttk.Checkbutton(lt, text="Parar en esquinas y suavizar escalones",
                            variable=self.var_esquinas).grid(row=2, column=0, columnspan=4, sticky="w")
            self.lbl_cond = ttk.Label(lt, wraplength=280, justify="left", foreground="#555")
            self.lbl_cond.grid(row=3, column=0, columnspan=4, sticky="w", pady=(3, 0))
            self._mostrar_condiciones()

        o = ttk.LabelFrame(side, text="Archivo de salida", padding=6)
        o.pack(fill="x", pady=3)
        ttk.Combobox(o, textvariable=self.var_fmt, state="readonly", width=34,
                     values=list(FORMATOS)).pack(fill="x")

        b = ttk.Frame(side)
        b.pack(fill="x", pady=6)
        btns = [("Procesar  (Enter)", self.process), ("Simular trayectoria", self.simulate),
                ("Guardar .txt  (Ctrl+S)", self.save), ("Abrir .txt  (Ctrl+O)", self.open_txt),
                ("Deshacer  (Ctrl+Z)", self.undo), ("Limpiar  (Supr)", self.clear),
                ("Exportar para la pierna  (Ctrl+E)", self.export_leg)]
        for i, (t, c) in enumerate(btns):
            ttk.Button(b, text=t, command=c).grid(row=i // 2, column=i % 2, sticky="ew", padx=2, pady=2,
                                                  columnspan=2 if i == len(btns) - 1 else 1)
        b.columnconfigure((0, 1), weight=1)

        ttk.Label(side, textvariable=self.info, wraplength=280, justify="left",
                  foreground="#1a4f8a").pack(fill="x", pady=3)
        ttk.Label(side, text="Vista previa del archivo:").pack(anchor="w")
        self.preview = tk.Text(side, width=34, height=10, font=("monospace", 9))
        self.preview.pack(fill="both", expand=True)

        ttk.Label(main, textvariable=self.status, relief="sunken", anchor="w").grid(
            row=1, column=0, columnspan=2, sticky="ew", pady=(6, 0))

        self.bind("<Control-z>", lambda e: self.undo())
        self.bind("<Control-s>", lambda e: self.save())
        self.bind("<Control-o>", lambda e: self.open_txt())
        self.bind("<Control-e>", lambda e: self.export_leg())
        self.bind("<Return>", lambda e: self.process())
        self.bind("<Delete>", lambda e: self.clear())

    # ------------------------------------------------------- coordenadas ----
    def _set_area(self):
        try:
            self.W = max(1.0, float(self.var_w.get()))
            self.H = max(1.0, float(self.var_h.get()))
        except (tk.TclError, ValueError):
            messagebox.showerror("Plano", "Dimensiones no válidas")
            return
        self.scale = CANVAS_MAX_PX / max(self.W, self.H)
        self.canvas.config(width=self.W * self.scale + 2 * PAD,
                           height=self.H * self.scale + 2 * PAD)
        self.process(silent=True)

    def to_px(self, x, y):
        return PAD + x * self.scale, PAD + (self.H - y) * self.scale

    def to_mm(self, px, py):
        x = (px - PAD) / self.scale
        y = self.H - (py - PAD) / self.scale
        return min(max(x, 0.0), self.W), min(max(y, 0.0), self.H)

    # ----------------------------------------------------------- dibujo -----
    def _on_press(self, e):
        self._stop_sim()
        if self.loaded:
            self.loaded, self.path, self.figs = False, [], []
            self.redraw()
        self.current = [self.to_mm(e.x, e.y)]

    def _on_drag(self, e):
        if self.current is None:
            return
        p = self.to_mm(e.x, e.y)
        a = self.to_px(*self.current[-1])
        b = self.to_px(*p)
        self.canvas.create_line(*a, *b, width=3, fill="#222", capstyle="round", tags="live")
        self.current.append(p)
        self._on_motion(e)

    def _on_release(self, e):
        if self.current and len(self.current) > 1:
            self.strokes.append(self.current)
        self.current = None
        self.process(silent=True)

    def _on_motion(self, e):
        x, y = self.to_mm(e.x, e.y)
        self.status.set(f"x = {x:6.1f} mm   y = {y:6.1f} mm      "
                        f"plano {self.W:g} x {self.H:g} mm   trazos: {len(self.strokes)}")

    def undo(self):
        if self.strokes:
            self.strokes.pop()
        self.process(silent=True)

    def clear(self):
        self._stop_sim()
        self.strokes, self.figs, self.path, self.loaded = [], [], [], False
        self._check_reach()
        self.redraw()
        self._update_info()

    # ------------------------------------------------------ procesamiento ---
    def _schedule_process(self):
        if self._proc_job:
            self.after_cancel(self._proc_job)
        self._proc_job = self.after(150, lambda: self.process(silent=True))

    def _params(self):
        step = float(self.var_step.get())
        dec = int(self.var_dec.get())
        z, lift = float(self.var_z.get()), float(self.var_lift.get())
        if not (0.01 <= step <= 1000) or not (0 <= dec <= 6):
            raise ValueError
        return step, dec, z, lift

    def process(self, silent=False):
        self._proc_job = None
        if self.loaded:
            self._check_reach()
            self.redraw()
            return
        try:
            step, dec, z, lift = self._params()
        except (tk.TclError, ValueError):
            if not silent:
                messagebox.showerror("Parámetros", "Paso, decimales o Z no válidos")
            return
        mode = "libre" if self.var_mode.get() == "Trazo libre" else "figura"
        self.figs = []
        for s in self.strokes:
            f = analyze_stroke(s, mode, float(self.var_sens.get()),
                               self.var_close.get(), step)
            if f:
                self.figs.append(f)
        self.path = build_path(self.figs, self.W, self.H, step, dec, self.var_norm.get(),
                               z, lift)
        self._check_reach()
        self.redraw()
        self._update_info()

    def _side(self):
        return LADOS.get(self.var_side.get(), "left")

    def _set_side_defaults(self):
        rec = PAPEL_RECOMENDADO[self._side()]
        self.var_xp.set(rec["x_paper"])
        self.var_yc.set(rec["yc"])
        self.var_zc.set(rec["zc"])
        self.var_w.set(rec["lado"])
        self.var_h.set(rec["lado"])
        self._set_area()
        self.var_leg_lift.set(20)
        self.var_scale.set(1)

    def _leg_params(self):
        scale, lift = float(self.var_scale.get()), float(self.var_leg_lift.get())
        if scale <= 0:
            raise ValueError
        # El centro del plano de trabajo del trazador cae en (yc, zc).
        return dict(x_paper=float(self.var_xp.get()), yc=float(self.var_yc.get()),
                    zc=float(self.var_zc.get()), width=self.W, height=self.H,
                    scale=scale, mirror=self.var_mirror.get(), lift=lift)

    def _check_reach(self):
        self.bad = set()
        if self.ik is None:
            return
        if not self.var_check.get() or not self.path:
            self.reach.set("")
            return
        try:
            leg = paper_to_leg(self.path, **self._leg_params())
        except (tk.TclError, ValueError):
            self.reach.set("Parámetros del marco no válidos")
            self.lbl_reach.config(foreground="#d62728")
            return
        self.bad = set(unreachable_points(leg, self.ik, self._side()))
        if self.bad:
            self.reach.set(f"✗ {len(self.bad)} de {len(leg)} puntos fuera de alcance "
                           "(en rojo). Prueba otra altura, otro centro o una escala menor.")
            self.lbl_reach.config(foreground="#d62728")
        else:
            self.reach.set(f"✓ Los {len(leg)} puntos son alcanzables.")
            self.lbl_reach.config(foreground="#2ca02c")

    def _update_info(self):
        if not self.path:
            self.info.set("Sin trayectoria")
        elif self.loaded:
            self.info.set(f"Archivo cargado: {len(self.path)} puntos   ·   "
                          f"levantamientos de lápiz: {count_lifts(self.path)}")
        else:
            names = ", ".join(f["name"] for f in self.figs)
            n_v = sum(len(f["verts"]) for f in self.figs if not f["name"].startswith("círculo"))
            self.info.set(f"Figura(s): {names}\nPuntos interpolados: {len(self.path)}"
                          + (f"   ·   vértices: {n_v}" if n_v else "")
                          + f"\nLevantamientos de lápiz: {count_lifts(self.path)}")
        self.preview.delete("1.0", "end")
        if self.path:
            txt = format_path(self.path[:400], FORMATOS[self.var_fmt.get()], self._dec())
            if len(self.path) > 400:
                txt += "...\n"
            self.preview.insert("1.0", txt)

    def _dec(self):
        if self.loaded:
            return 0 if all(float(v).is_integer() for p in self.path for v in p) else 3
        try:
            return int(self.var_dec.get())
        except (tk.TclError, ValueError):
            return 0

    # ------------------------------------------------------------ dibujo ----
    def redraw(self):
        c = self.canvas
        c.delete("all")
        x0, y0 = self.to_px(0, self.H)
        x1, y1 = self.to_px(self.W, 0)
        c.create_rectangle(x0, y0, x1, y1, fill="white", outline="#888")
        g = GRID_MM
        while g * self.scale < 25:
            g *= 2
        k = 0
        while k <= self.W + 1e-9:
            px, _ = self.to_px(k, 0)
            c.create_line(px, y0, px, y1, fill="#e6e6e6")
            c.create_text(px, y1 + 12, text=f"{k:g}", fill="#777", font=("sans", 8))
            k += g
        k = 0
        while k <= self.H + 1e-9:
            _, py = self.to_px(0, k)
            c.create_line(x0, py, x1, py, fill="#e6e6e6")
            c.create_text(x0 - 14, py, text=f"{k:g}", fill="#777", font=("sans", 8))
            k += g
        c.create_text(x1, y1 + 26, text="X (mm)", anchor="e", fill="#555", font=("sans", 8))
        c.create_text(x0 - 4, y0 - 14, text="Y (mm)", anchor="w", fill="#555", font=("sans", 8))
        c.create_rectangle(x0, y0, x1, y1, outline="#888")

        stroke_col = "#c8c8c8" if self.path else "#222"
        for s in self.strokes:
            if len(s) > 1:
                c.create_line(*[v for p in s for v in self.to_px(*p)], width=3,
                              fill=stroke_col, capstyle="round", joinstyle="round")

        if not self.path:
            return
        # Si se normalizó, la trayectoria se dibuja tal y como saldrá en el archivo.
        # Tramos con el lápiz abajo en azul; desplazamientos en el aire en gris
        # discontinuo (proyección XY: la subida/bajada vertical no se ve).
        zd = self.path[0][2]
        pts = [self.to_px(p[0], p[1]) for p in self.path]
        down = [p[2] == zd for p in self.path]
        run = [pts[0]]
        for i in range(1, len(pts)):
            seg_down = down[i - 1] and down[i]
            prev_down = down[i - 2] and down[i - 1] if i > 1 else seg_down
            if seg_down != prev_down:
                self._draw_run(run, prev_down)
                run = [pts[i - 1]]
            run.append(pts[i])
        if len(pts) > 1:
            self._draw_run(run, down[-2] and down[-1])
        for i in range(len(pts) - 1):
            if down[i] != down[i + 1]:   # punto donde sube o baja el lápiz
                x, y = pts[i] if down[i] else pts[i + 1]
                c.create_text(x, y - 9, text="▲" if down[i] else "▼",
                              fill="#888", font=("sans", 8))
        if self.var_show_pts.get() and len(pts) < 5000:
            for (x, y), d in zip(pts, down):
                if d:
                    c.create_oval(x - 1.5, y - 1.5, x + 1.5, y + 1.5, fill="#1f6fd1", outline="")
        if not self.loaded and not self.var_norm.get():
            for f in self.figs:
                if not f["name"].startswith("círculo"):
                    for v in f["verts"]:
                        x, y = self.to_px(*v)
                        c.create_oval(x - 4, y - 4, x + 4, y + 4, outline="#d62728", width=2)
                ys = [v[1] for v in f["verts"]]
                xs = [v[0] for v in f["verts"]]
                tx, ty = self.to_px(sum(xs) / len(xs), max(ys))
                c.create_text(tx, ty - 12, text=f["name"], fill="#d62728", font=("sans", 9, "bold"))
        for i in self.bad:
            x, y = pts[i]
            c.create_oval(x - 3, y - 3, x + 3, y + 3, fill="#d62728", outline="")
        sx, sy = pts[0]
        c.create_oval(sx - 5, sy - 5, sx + 5, sy + 5, fill="#2ca02c", outline="")
        c.create_text(sx + 8, sy + 8, text="inicio", anchor="nw", fill="#2ca02c", font=("sans", 8))

    def _draw_run(self, run, pen_down):
        if len(run) < 2:
            return
        flat = [v for p in run for v in p]
        if pen_down:
            self.canvas.create_line(*flat, fill="#1f6fd1", width=2)
        else:
            self.canvas.create_line(run[0][0], run[0][1], run[-1][0], run[-1][1],
                                    fill="#999", width=1, dash=(5, 4))

    # -------------------------------------------------------- simulación ----
    def _stop_sim(self):
        if self.sim_job:
            self.after_cancel(self.sim_job)
            self.sim_job = None
        self.canvas.delete("sim")

    def _ley(self):
        inv = {v: k for k, v in self.perf.NOMBRES.items()}
        return inv.get(self.var_ley.get(), "quintico")

    def _mostrar_condiciones(self):
        """Qué fija la ley elegida y qué parámetros usa (los demás se
        desactivan)."""
        texto, usa = self.perf.CONDICIONES_CAMINO[self._ley()]
        self.lbl_cond.config(text=texto)
        self.sp_v.state(["!disabled"] if "v" in usa else ["disabled"])
        self.sp_a.state(["!disabled"] if "a" in usa else ["disabled"])

    def _timeline(self):
        """Muestras (cada 20 ms) del recorrido del lápiz con la ley temporal
        elegida: (posiciones Nx3 en el papel, lápiz abajo N)."""
        P = [tuple(p) for p in self.path]
        zd = P[0][2]
        metodo, v, a = self._ley(), float(self.var_v_lapiz.get()), float(self.var_a_lapiz.get())
        esquinas = self.var_esquinas.get()
        pos, abajo = [], []
        i = 0
        while i < len(P):
            j = i
            down = P[i][2] == zd
            while j + 1 < len(P) and (P[j + 1][2] == zd) == down:
                j += 1
            # el tramo incluye el punto de unión con el siguiente
            tramo = P[i:j + 2] if j + 1 < len(P) else P[i:j + 1]
            if len(tramo) >= 2:
                lados = [tramo]
                if down and esquinas:
                    c = self.perf.dividir_en_esquinas(tramo)
                    lados = [self.perf.suavizar_lado(tramo[x:y + 1]) for x, y in zip(c, c[1:])]
                for lado in lados:
                    _, muestras = self.perf.ley_temporal_camino(lado, metodo, v, a, 0.02)
                    pos.extend(muestras)
                    abajo.extend([down] * len(muestras))
            i = j + 1
        return pos, abajo

    def simulate(self):
        self._stop_sim()
        if not self.path:
            return
        if self.perf is None:
            return self._simulate_fijo()
        try:
            pos, abajo = self._timeline()
        except (self.perf.ErrorPerfil, ValueError, tk.TclError) as error:
            messagebox.showerror("Simular", f"Ley temporal no válida: {error}")
            return
        pts = [self.to_px(p[0], p[1]) for p in pos]
        T = 0.02 * (len(pts) - 1)
        t_abajo = 0.02 * sum(abajo)
        t0 = time.monotonic()
        estado = {"i": 0}

        def step():
            i_obj = min(len(pts) - 1, int((time.monotonic() - t0) / 0.02))
            i = estado["i"]
            while i < i_obj:
                if abajo[i] and abajo[i + 1]:
                    self.canvas.create_line(*pts[i], *pts[i + 1], fill="#ff7f0e", width=4,
                                            capstyle="round", tags="sim")
                i += 1
            estado["i"] = i
            self.canvas.delete("tip")
            x, y = pts[i]
            self.canvas.create_oval(x - 6, y - 6, x + 6, y + 6,
                                    fill="#ff7f0e" if abajo[i] else "",
                                    outline="black", width=1 if abajo[i] else 2, tags=("sim", "tip"))
            self.status.set(f"Simulando ({self.var_ley.get()}): {0.02 * i:5.1f} / {T:.1f} s"
                            f"   ·   lápiz abajo {t_abajo:.1f} s"
                            + ("" if abajo[i] else "   (lápiz arriba)"))
            if i >= len(pts) - 1:
                self.sim_job = None
                return
            self.sim_job = self.after(20, step)
        step()

    def _simulate_fijo(self):
        path = self.path
        zd = path[0][2]
        pts = [self.to_px(p[0], p[1]) for p in path]
        delay = max(1, min(20, 4000 // len(pts)))

        def step(i):
            if i >= len(pts):
                self.sim_job = None
                return
            down = path[i][2] == zd
            if i > 0 and down and path[i - 1][2] == zd:
                self.canvas.create_line(*pts[i - 1], *pts[i], fill="#ff7f0e", width=4,
                                        capstyle="round", tags="sim")
            self.canvas.delete("tip")
            x, y = pts[i]
            self.canvas.create_oval(x - 6, y - 6, x + 6, y + 6,
                                    fill="#ff7f0e" if down else "",
                                    outline="black", width=1 if down else 2, tags=("sim", "tip"))
            self.status.set(f"Simulando punto {i + 1}/{len(path)}:  "
                            + ", ".join(f"{c:g}" for c in path[i])
                            + ("" if down else "   (lápiz arriba)"))
            self.sim_job = self.after(delay, step, i + 1)
        step(0)

    # --------------------------------------------------------- archivos -----
    def save(self):
        if not self.loaded:
            self.process(silent=True)
        if not self.path:
            messagebox.showinfo("Guardar", "No hay ninguna trayectoria que guardar.")
            return
        fn = filedialog.asksaveasfilename(defaultextension=".txt",
                                          filetypes=[("Texto", "*.txt"), ("Todos", "*")],
                                          initialfile="trayectoria.txt")
        if not fn:
            return
        with open(fn, "w", encoding="utf-8") as fh:
            fh.write(format_path(self.path, FORMATOS[self.var_fmt.get()], self._dec()))
        self.status.set(f"Guardado {fn}  ({len(self.path)} puntos)")

    def open_txt(self):
        fn = filedialog.askopenfilename(filetypes=[("Texto", "*.txt"), ("Todos", "*")])
        if not fn:
            return
        with open(fn, encoding="utf-8") as fh:
            try:
                z_def = float(self.var_z.get())
            except (tk.TclError, ValueError):
                z_def = 200.0
            path = parse_path(fh.read(), z_def)
        if not path:
            messagebox.showerror("Abrir", "No se encontraron coordenadas en el archivo.")
            return
        self._stop_sim()
        self.strokes, self.figs, self.path, self.loaded = [], [], path, True
        self._check_reach()
        self.redraw()
        self._update_info()
        self.status.set(f"Cargado {fn}")

    def export_leg(self):
        if not self.loaded:
            self.process(silent=True)
        if not self.path:
            messagebox.showinfo("Exportar", "No hay ninguna trayectoria que exportar.")
            return
        try:
            params = self._leg_params()
        except (tk.TclError, ValueError):
            messagebox.showerror("Exportar", "Parámetros del marco de la pierna no válidos")
            return
        if self.bad and not messagebox.askyesno(
                "Exportar", f"{len(self.bad)} puntos están fuera del alcance de la pierna.\n"
                            "La interfaz se moverá de forma incorrecta. ¿Exportar de todos modos?"):
            return
        fn = filedialog.asksaveasfilename(
            defaultextension=".txt", filetypes=[("Texto", "*.txt"), ("Todos", "*")],
            initialdir=LEG_TRAJ_DIR if os.path.isdir(LEG_TRAJ_DIR) else None,
            initialfile="trayectoria_pierna.txt")
        if not fn:
            return
        leg = paper_to_leg(self.path, **params)
        ley = ""
        if self.perf is not None:
            ley = (f"\nley_temporal metodo={self._ley()} v={float(self.var_v_lapiz.get()):g} "
                   f"a={float(self.var_a_lapiz.get()):g} "
                   f"esquinas={'si' if self.var_esquinas.get() else 'no'}")
        header = (f"Trayectoria exportada desde trazador.py (pierna {self.var_side.get().lower()}, "
                  "sistema de las pestañas de IK: X hacia abajo, Y adelante, Z lateral, mm)\n"
                  + "x_papel={x_paper:g} centro=({yc:g}, {zc:g}) papel={width:g}x{height:g} "
                    "escala={scale:g} levantar={lift:g} espejo={mirror}".format(**params)
                  + ley)
        with open(fn, "w", encoding="utf-8") as fh:
            fh.write(format_leg_path(leg, header))
        self.status.set(f"Exportado {fn}  ({len(leg)} puntos)")


if __name__ == "__main__":
    Trazador().mainloop()

