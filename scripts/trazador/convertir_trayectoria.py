#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Convierte una trayectoria del trazador (coordenadas del PAPEL) al sistema
de coordenadas de la PIERNA que usan las pestañas de cinemática inversa
(X a lo largo de la pierna hacia abajo, Y adelante, Z lateral; el papel
horizontal es un plano X = constante).

Papel (trazador.py):  origen en la esquina inferior izquierda del dibujo,
                      u hacia la derecha, v hacia arriba, w = altura del lápiz
                      (plano de dibujo + elevación al levantar), en mm.

Pierna:               ver paper_to_leg en trazador.py, que hace la
                      conversión.

Útil para trayectorias guardadas con "Guardar .txt" (coordenadas del papel).
El dibujo queda centrado en el área de escritura recomendada para la pierna
elegida (docs/ESPACIO_TRABAJO_Y_SUPERFICIE_ESCRITURA.md).

Uso:
    python3 convertir_trayectoria.py entrada.txt salida.txt
    python3 convertir_trayectoria.py entrada.txt salida.txt --pierna right --escala 0.5
"""
import argparse

from trazador import PAPEL_RECOMENDADO, format_leg_path, paper_to_leg, parse_path


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("entrada")
    ap.add_argument("salida")
    ap.add_argument("--pierna", choices=("left", "right"), default="left")
    ap.add_argument("--x", type=float, default=None,
                    help="X del papel (a lo largo de la pierna, hacia abajo) [mm]")
    ap.add_argument("--yc", type=float, default=None, help="Y (adelante) del centro del dibujo [mm]")
    ap.add_argument("--zc", type=float, default=None, help="Z (lateral) del centro del dibujo [mm]")
    ap.add_argument("--levantar", type=float, default=20.0,
                    help="cuánto se levanta el lápiz entre figuras (mm)")
    ap.add_argument("--escala", type=float, default=1.0)
    ap.add_argument("--espejo", action="store_true",
                    help="invierte izquierda/derecha (si el dibujo se ve al revés en RViz)")
    a = ap.parse_args()

    rec = PAPEL_RECOMENDADO[a.pierna]
    x_paper = rec["x_paper"] if a.x is None else a.x
    yc = rec["yc"] if a.yc is None else a.yc
    zc = rec["zc"] if a.zc is None else a.zc

    with open(a.entrada, encoding="utf-8") as f:
        pts = parse_path(f.read())
    if not pts:
        raise SystemExit(f"No se encontraron puntos en {a.entrada}")

    # El dibujo se centra por su propio contorno, no por el plano del trazador.
    us = [p[0] for p in pts]
    vs = [p[1] for p in pts]
    u0, v0 = min(us), min(vs)
    width, height = max(us) - u0, max(vs) - v0
    pts = [(u - u0, v - v0, w) for u, v, w in pts]

    leg = paper_to_leg(pts, x_paper, yc, zc, width, height, a.escala, a.espejo, a.levantar)
    header = (f"Convertido desde {a.entrada} (pierna {a.pierna}, sistema de las pestañas de IK: "
              f"X hacia abajo, Y adelante, Z lateral, mm)\n"
              f"x_papel={x_paper:g} centro=({yc:g}, {zc:g}) escala={a.escala:g} "
              f"levantar={a.levantar:g} espejo={a.espejo}")
    with open(a.salida, "w", encoding="utf-8") as f:
        f.write(format_leg_path(leg, header))

    print(f"{len(pts)} puntos -> {a.salida}")


if __name__ == "__main__":
    main()
