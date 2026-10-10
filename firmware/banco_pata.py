#!/usr/bin/env python3
"""
Banco de Pata: interfaz grafica de la pata robotica
(brushless WK8115-4 + cicloidal 1:11 + moteus-c1, y brazo con dos pares de servos)
ESP32 + MCP2518FD (CAN) + PCA9685 (servos) + TCA9548A con dos AS5600 (encoders)

Muestra la pata en 3D, el mando del brushless, los dos pares de servos (arriba y abajo)
con sus encoders, bateria, temperaturas, corrientes, graficas de 20 s y la consola serie.
Usa la telemetria del main.cpp (comando p1, lineas "@T,..." y "@S,...").
Cada mando envia solo su comando: el brushless, el par de arriba y el par de abajo
se mueven de forma independiente.

Instalar (una sola vez):
    pip install pyside6 pyqtgraph pyopengl pyserial numpy

Ejecutar:
    python3 banco_pata.py                 # abre en simulacion; conecta desde la ventana
    python3 banco_pata.py /dev/ttyUSB0    # intenta conectar directo a ese puerto

Cierra el monitor de PlatformIO antes de conectar: solo un programa puede usar el puerto.
"""
import math
import random
import sys
import threading
import time
from collections import deque

import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl
import serial
import serial.tools.list_ports
from PySide6 import QtCore, QtGui, QtWidgets

# ---------------------------------------------------------------- colores
C = {
    "bg": "#0e1318", "panel": "#151c23", "panel2": "#1b242d", "line": "#27333e",
    "fg": "#e3e9ee", "fg2": "#a9b5bf", "muted": "#7d8b97",
    "accent": "#e07a3f", "real": "#d9733a", "target": "#2f9cc4",
    "ok": "#45b97c", "warn": "#e2b23c", "crit": "#e5534b", "viewport": "#10171d",
    "arriba": "#a07ee0", "abajo": "#33a088",
}
FUENTE = "IBM Plex Sans, Inter, Segoe UI, Ubuntu, DejaVu Sans, sans-serif"
MONO = "IBM Plex Mono, JetBrains Mono, Ubuntu Mono, DejaVu Sans Mono, monospace"

# ---------------------------------------------------------------- tablas del moteus
MODOS = {0: "Detenido", 1: "Falla", 2: "Habilitando", 3: "Calibrando", 4: "Calibracion completa",
         5: "PWM", 6: "Voltaje", 7: "Voltaje FOC", 8: "Voltaje dq", 9: "Corriente", 10: "Posicion",
         11: "Tiempo agotado", 12: "Velocidad cero", 13: "Mantener rango", 14: "Medir inductancia", 15: "Freno"}
FALLAS = {
    32: ("Falla de calibracion", "Repite la calibracion del motor."),
    33: ("Falla del driver (gate driver)", "Revisa las tres fases: un falso contacto produjo esta falla antes."),
    34: ("Sobrevoltaje", "El bus subio demasiado, normalmente al frenar. Revisa la fuente o bateria."),
    35: ("Falla del encoder", "Revisa el AS5047P y el cable de AUX2."),
    36: ("Motor sin configurar", "Falta la calibracion del motor en el moteus."),
    37: ("Sobrecarga del ciclo PWM", "Ciclo de control demasiado pesado (se vio antes con SPI rapido)."),
    38: ("Sobretemperatura", "Deja enfriar la placa antes de seguir."),
    39: ("Arranque fuera de limites", "La posicion esta fuera de position_min/max."),
    40: ("Bajo voltaje", "Bateria descargada o fuente insuficiente."),
    41: ("Configuracion cambiada", "Envia stop y vuelve a mandar el comando."),
    42: ("Angulo electrico invalido", "Problema con la conmutacion del encoder."),
    43: ("Posicion invalida", "La posicion de salida no esta inicializada."),
    44: ("Falla al habilitar el driver", "Revisa la alimentacion del driver."),
    46: ("Violacion de tiempo", "Revisa la configuracion de la placa."),
    48: ("Limites invalidos", "Revisa los limites del comando."),
    96: ("Limite de velocidad", "servo.max_velocity esta limitando."),
    97: ("Limite de potencia", "servo.max_power_W esta limitando."),
    98: ("Limite de voltaje", "El voltaje del bus esta limitando."),
    99: ("Limite de corriente", "servo.max_current_A (6 A) esta limitando."),
    100: ("Limitado por temperatura de placa", "La placa esta caliente y reduce corriente."),
    101: ("Limitado por temperatura del motor", "El motor esta caliente y reduce corriente."),
    102: ("Limite de par del comando", "El par pedido supera el limite (comando t). Subelo si la pata no llega."),
    103: ("Limite de posicion", "servo.position_min/max esta limitando."),
}
LIPO = [(3.27, 0), (3.61, 5), (3.69, 10), (3.71, 15), (3.73, 20), (3.75, 25), (3.77, 30), (3.79, 35),
        (3.80, 40), (3.82, 45), (3.84, 50), (3.85, 55), (3.87, 60), (3.91, 65), (3.95, 70), (3.98, 75),
        (4.02, 80), (4.08, 85), (4.11, 90), (4.15, 95), (4.20, 100)]

CAMPOS = ["ms", "modo", "fault", "ang", "dest", "vel", "par", "ff", "iq", "id", "volt", "temp", "mtemp",
          "potencia", "tmax", "grav", "banderas", "can_ok", "can_env", "motor_rev"]
CAMPOS_S = ["ms", "a_dest", "a_cmd", "a_enc", "a_raw", "a_b", "b_dest", "b_cmd", "b_enc", "b_raw", "b_b",
            "i1", "i2", "i3"]
PARES = (("a", "Arriba"), ("b", "Abajo"))
VENTANA_S = 20.0


def nombre_falla(f):
    return "Sin fallas" if f == 0 else FALLAS.get(f, (f"Codigo {f}", ""))[0]


def lipo_pct(v_celda):
    if v_celda <= LIPO[0][0]:
        return 0.0
    for (v0, p0), (v1, p1) in zip(LIPO, LIPO[1:]):
        if v_celda <= v1:
            return p0 + (p1 - p0) * (v_celda - v0) / (v1 - v0)
    return 100.0


def fmt(v, d, u=""):
    if v is None or (isinstance(v, float) and math.isnan(v)):
        return "--"
    return f"{v:.{d}f}" + (f" {u}" if u else "")


# ---------------------------------------------------------------- puerto serie
class PuertoSerie(QtCore.QObject):
    linea = QtCore.Signal(str)
    perdido = QtCore.Signal(str)

    def __init__(self, nombre):
        super().__init__()
        # DTR/RTS en reposo antes de abrir: asi el ESP32 no se reinicia al conectar
        # (el reinicio mueve servos de golpe y el pico de corriente tumba el USB)
        self.ser = serial.Serial(baudrate=115200, timeout=0.1)
        self.ser.port = nombre
        self.ser.dtr = False
        self.ser.rts = False
        self.ser.open()
        self._activo = True
        self._hilo = threading.Thread(target=self._leer, daemon=True)
        self._hilo.start()

    def _leer(self):
        buf = b""
        while self._activo:
            try:
                datos = self.ser.read(512)
            except (serial.SerialException, OSError) as e:
                if self._activo:
                    self.perdido.emit(str(e))
                return
            if not datos:
                continue
            buf += datos
            while b"\n" in buf:
                fila, buf = buf.split(b"\n", 1)
                self.linea.emit(fila.decode(errors="ignore").rstrip("\r"))
            if len(buf) > 4000:
                buf = b""

    def enviar(self, texto):
        self.ser.write((texto + "\n").encode())

    def cerrar(self):
        try:
            self.enviar("p0")
        except Exception:
            pass
        self._activo = False
        time.sleep(0.15)
        try:
            self.ser.close()
        except Exception:
            pass


# ---------------------------------------------------------------- simulador
class Simulador(QtCore.QObject):
    """Imita al ESP32: responde los mismos comandos y envia la misma telemetria."""
    linea = QtCore.Signal(str)

    J, G, VMAX, AMAX = 0.25, 5.7, 54.0, 108.0

    def __init__(self):
        super().__init__()
        self.th = -90.0; self.w = 0.0; self.cmd_p = -90.0; self.cmd_v = 0.0; self.target = 30.0
        self.pos = True; self.verificado = True; self.tmax = 30.0; self.grav = 5.7; self.tele = True
        self.ff = 0.0; self.par = 0.0; self.iq = 0.0; self.volt = 24.7; self.tb = 31.0; self.tm = 29.0
        self.ok = 0; self.env = 0; self.moviendo = True; self.quieto = 0.0; self.t_ini = 0.0; self.reloj = 0.0
        self.midiendo = 0.0; self.suma = 0.0; self.n = 0; self.fault = 0
        # servos: arriba arranca moviendose, abajo quieto
        self.sv = [dict(dest=30.0, cmd=-50.0, enc=-50.0, activo=True, mov=True, vel=20.0, cero=1000),
                   dict(dest=-20.0, cmd=-20.0, enc=-20.0, activo=True, mov=False, vel=20.0, cero=2500)]
        self.timer = QtCore.QTimer(self); self.timer.timeout.connect(self._tick)
        self._ultimo = time.monotonic()

    def iniciar(self):
        self._ultimo = time.monotonic()
        self.timer.start(40)
        self.linea.emit("ESP32 + MCP2518FD + moteus-c1 (pata) [simulacion]")
        self.linea.emit("Calibracion cargada. Cero confirmado. Ir a 30.0 grados")
        self.linea.emit("Servos Arriba a 30.0 grados")

    def detener(self):
        self.timer.stop()

    def _reporte(self, titulo):
        for t in (titulo, f"  Destino:  {self.target:.1f} grados", f"  Posicion: {self.th:.2f} grados",
                  f"  Error:    {self.target - self.th:.2f} grados", f"  Par:      {self.par:.2f} Nm",
                  f"  Fault:    {self.fault}", f"  Tiempo:   {self.reloj - self.t_ini:.1f} s"):
            self.linea.emit(t)

    def enviar(self, txt):
        o = self.linea.emit
        txt = txt.strip()
        if not txt:
            return
        c = txt[0].lower()
        if c in "ab" and len(txt) > 1:
            self._servos(0 if c == "a" else 1, txt); return
        if c.isdigit() or c in ".-":
            try:
                a = float(txt)
            except ValueError:
                o("Comando no reconocido: " + txt); return
            if not self.verificado:
                o("Verifica el cero primero (v o z)."); return
            if not -90 <= a <= 90:
                o("Angulo fuera de rango."); o("  Usa un valor entre -90 y 90."); return
            if not self.pos:
                self.cmd_p, self.cmd_v = self.th, self.w
            self.target, self.pos, self.moviendo, self.quieto, self.t_ini = a, True, True, 0.0, self.reloj
            o(f"Ir a {a:.1f} grados"); return
        if c == "t" and len(txt) > 1:
            try:
                t = float(txt[1:])
            except ValueError:
                t = -1
            if 0 < t <= 40:
                self.tmax = t; o(f"Limite de par: {t:.1f} Nm")
            else:
                o("Usa un valor entre 0 y 40 (ej: t30)")
            return
        if c == "p" and len(txt) == 2:
            self.tele = txt[1] == "1"; o("Telemetria activada" if self.tele else "Telemetria en pausa"); return
        if c == "s":
            self.pos = False; self.moviendo = False; o("Stop brushless: motor libre (sosten la pata).")
        elif c == "x":
            self.pos = False; self.moviendo = False
            for sv in self.sv:
                sv["activo"] = sv["mov"] = False
            o("PARAR TODO: brushless libre y servos sueltos.")
        elif c == "i":
            o("Sensores de corriente en cero.")
        elif c == "v":
            self.verificado = True; o("Cero confirmado. Ya puedes enviar angulos.")
        elif c == "z":
            if self.pos:
                o("Envia s primero y deja"); o("la pata colgando quieta.")
            else:
                self.verificado = True; o("CERO guardado (pata abajo):"); o("  0.1234 vueltas del motor (simulado)")
        elif c == "h":
            if self.pos:
                o("Primero: s y z con la pata abajo."); o("Luego pata horizontal y h.")
            else:
                o("SENTIDO guardado: signo 1"); o(f"  Recorrido medido: {abs(self.th + 90):.1f} grados (debe ser ~90, de colgando a horizontal)")
        elif c == "m":
            if not self.pos or abs(self.target) > 1:
                o("Envia 0 (horizontal), espera que"); o("quede quieta y luego envia m.")
            else:
                self.midiendo, self.suma, self.n = self.reloj, 0.0, 0
                o("Midiendo el par (3 s)."); o("No toques la pata...")
        elif c == "e":
            for t in ("Estado:", f"  Modo:     {10 if self.pos else 0}   Fault: {self.fault}",
                      f"  Pata:     {self.th:.2f} grados", f"  Par:      {self.par:.2f} Nm",
                      f"  Bateria:  {self.volt:.1f} V", f"  Temp:     {self.tb:.1f} C", f"  CAN:      {self.ok}/{self.env}"):
                o(t)
            for (letra, nombre), sv in zip(PARES, self.sv):
                o(f"Servos {nombre}: " + ("moviendose" if sv["mov"] else "quietos" if sv["activo"] else "sueltos"))
                o(f"  Destino {sv['dest']:.1f}  Comando {sv['cmd']:.1f} grados  Encoder {sv['enc']:.1f}")
        elif c == "c":
            for t in ("Calibracion ESP32:", "  Cero:     0.1234 vueltas (OK)", "  Sentido:  signo 1 (OK)",
                      f"  Gravedad: {self.grav:.2f} Nm en la horizontal (0 grados)", f"  Par max:  {self.tmax:.1f} Nm",
                      "Moteus (tview): kp 200, kd 4, ki 50, ilimit 12, 6 A"):
                o(t)
        elif c == "r":
            o("Calibracion borrada (en la simulacion se restaura sola).")
        elif c == "g":
            o("Graficas Teleplot (no aplica en la simulacion)")
        else:
            o("Comando no reconocido: " + txt)

    def _servos(self, i, txt):
        o = self.linea.emit; sv = self.sv[i]; nombre = PARES[i][1]; c1 = txt[1].lower()
        if c1 == "x":
            sv["activo"] = sv["mov"] = False; o(f"Servos {nombre} sueltos (sin fuerza)."); return
        if c1 == "z":
            if not sv["activo"] or sv["mov"]:
                o(f"Mueve los servos {nombre} a una posicion conocida"); o("y espera que lleguen antes de alinear.")
            else:
                o(f"Encoder {nombre} alineado: raw {int(sv['cero'])} = {sv['cmd']:.1f} grados")
            return
        if c1 == "v" and len(txt) > 2:
            try:
                v = float(txt[2:])
            except ValueError:
                v = 0
            if 5 <= v <= 360:
                sv["vel"] = v; o(f"Velocidad servos {nombre}: {v:.0f} grados/s")
            else:
                o("Usa una velocidad entre 5 y 360 (ej: av60)")
            return
        try:
            a = float(txt[1:])
        except ValueError:
            o("Comando no reconocido: " + txt); return
        if not -90 <= a <= 90:
            o(f"Servos {nombre}: angulo fuera de rango."); o("  Usa un valor entre -90 y 90."); return
        if not sv["activo"]:
            sv["cmd"] = sv["enc"]; sv["activo"] = True
        sv["dest"] = a; sv["mov"] = True
        o(f"Servos {nombre} a {a:.1f} grados")

    def _tick(self):
        ahora = time.monotonic()
        dt = min(0.2, ahora - self._ultimo); self._ultimo = ahora
        n = max(1, round(dt / 0.04))
        for i in range(n):
            self._paso(dt / n, i == n - 1)

    def _paso(self, dt, emitir=True):
        self.reloj += dt; self.env += 1; self.ok += 1
        rnd = lambda: random.random() - 0.5
        rad = math.pi / 180
        if self.pos:
            d = self.target - self.cmd_p
            frenado = self.cmd_v ** 2 / (2 * self.AMAX)
            if abs(d) < 0.3 and abs(self.cmd_v) < 6:
                self.cmd_p, self.cmd_v, a = self.target, 0.0, 0.0
            elif abs(d) <= frenado and math.copysign(1, d) == math.copysign(1, self.cmd_v):
                a = -math.copysign(self.AMAX, self.cmd_v)
            else:
                a = math.copysign(self.AMAX, d)
            self.cmd_v = max(-self.VMAX, min(self.VMAX, self.cmd_v + a * dt))
            self.cmd_p += self.cmd_v * dt
            nuevo = self.th + (self.cmd_p - self.th) * min(1.0, dt / 0.06)
            self.w = (nuevo - self.th) / dt; self.th = nuevo
            self.ff = 0.0 if self.midiendo else self.grav * math.cos(self.th * rad)
            par = self.G * math.cos(self.th * rad) + self.J * a * rad + rnd() * 0.06
            self.fault = 0
            if abs(par) > self.tmax:
                par = math.copysign(self.tmax, par); self.fault = 102; self.th -= 0.02
            self.par = par
        else:
            alfa = -(self.G * math.cos(self.th * rad)) / self.J / rad - 1.6 * self.w
            self.w += alfa * dt; self.th += self.w * dt
            self.par = self.ff = 0.0; self.fault = 0; self.cmd_p, self.cmd_v = self.th, 0.0
        self.iq = self.par / 5.7 + rnd() * 0.02
        self.volt = max(19.0, self.volt - dt * 0.0004)
        v_ahora = self.volt - 0.12 * abs(self.iq) + rnd() * 0.03
        self.tb += dt * (0.05 * self.iq ** 2 - (self.tb - 27) * 0.004)
        self.tm += dt * (0.08 * self.iq ** 2 - (self.tm - 26) * 0.003)
        if self.midiendo:
            tt = self.reloj - self.midiendo
            if 1.5 < tt <= 3:
                self.suma += self.par; self.n += 1
            if tt > 3:
                self.midiendo = 0.0
                if self.n:
                    self.grav = abs(self.suma / self.n)
                self.linea.emit(f"Gravedad guardada: {self.grav:.2f} Nm")
        if self.moviendo and self.pos:
            if abs(self.target - self.th) < 1 and abs(self.w) < 1.8:
                self.quieto += dt
                if self.quieto > 0.5:
                    self._reporte("Llego:"); self.moviendo = False
            else:
                self.quieto = 0.0
            if self.moviendo and self.reloj - self.t_ini > 15:
                self._reporte("No llego. Revisa:"); self.moviendo = False
        corrientes = [0.05 + random.random() * 0.03] * 3
        for i, sv in enumerate(self.sv):
            if sv["activo"]:
                falta = sv["dest"] - sv["cmd"]; paso = sv["vel"] * dt
                if abs(falta) <= paso:
                    sv["cmd"] = sv["dest"]
                    if sv["mov"]:
                        sv["mov"] = False; sv["rep"] = self.reloj + 0.4
                    if sv.get("rep") and self.reloj >= sv["rep"]:
                        sv["rep"] = 0
                        nombre = PARES[i][1]
                        for t in (f"Servos {nombre} llegaron:", f"  Comando:  {sv['cmd']:.1f} grados",
                                  f"  Encoder:  {sv['enc']:.1f} grados", f"  Error:    {sv['cmd'] - sv['enc']:.1f} grados"):
                            self.linea.emit(t)
                else:
                    sv["cmd"] += math.copysign(paso, falta)
                sv["enc"] += (sv["cmd"] - sv["enc"]) * min(1.0, dt / 0.08) + rnd() * 0.15
                corrientes[i] = (0.35 if sv["activo"] else 0.02) + (0.9 if sv["mov"] else 0) + rnd() * 0.08
        if self.tele and emitir:
            campos = [f"@S,{int(self.reloj * 1000)}"]
            for sv in self.sv:
                b = (1 if sv["activo"] else 0) | 2 | 4 | 32 | (64 if sv["mov"] else 0)
                raw = int(sv["cero"] + sv["enc"] / 360 * 4096) % 4096
                campos.append(f"{sv['dest']:.1f},{sv['cmd']:.1f},{sv['enc']:.1f},{raw},{b}")
            campos.append(",".join(f"{x:.2f}" for x in corrientes))
            self.linea.emit(",".join(campos))
        if self.tele and emitir:
            band = 1 | 2 | (4 if self.verificado else 0) | (8 if self.pos else 0) | (16 if self.midiendo else 0) \
                | (32 if self.pos and self.cmd_p == self.target else 0) | (64 if self.moviendo else 0)
            self.linea.emit(
                f"@T,{int(self.reloj * 1000)},{10 if self.pos else 0},{self.fault},{self.th:.2f},{self.target:.2f},"
                f"{self.w:.2f},{self.par:.3f},{self.ff:.3f},{self.iq:.3f},{rnd() * 0.04:.3f},{v_ahora:.2f},"
                f"{self.tb:.1f},{self.tm:.1f},{v_ahora * abs(self.iq) * 0.35:.2f},{self.tmax:.1f},{self.grav:.2f},"
                f"{band},{self.ok},{self.env},{0.1234 + self.th / 360:.4f}")


# ---------------------------------------------------------------- piezas 3D
def malla_caja(sx, sy, sz):
    x, y, z = sx / 2, sy / 2, sz / 2
    v = np.array([[-x, -y, -z], [x, -y, -z], [x, y, -z], [-x, y, -z], [-x, -y, z], [x, -y, z], [x, y, z], [-x, y, z]])
    f = np.array([[0, 2, 1], [0, 3, 2], [4, 5, 6], [4, 6, 7], [0, 1, 5], [0, 5, 4],
                  [2, 3, 7], [2, 7, 6], [1, 2, 6], [1, 6, 5], [3, 0, 4], [3, 4, 7]])
    return gl.MeshData(vertexes=v, faces=f)


def malla_cilindro_y(r, largo, n=40):
    """Cilindro con tapas, eje a lo largo de Y (eje del motor)."""
    a = np.linspace(0, 2 * np.pi, n, endpoint=False)
    y0, y1 = -largo / 2, largo / 2
    anillo0 = np.c_[r * np.cos(a), np.full(n, y0), r * np.sin(a)]
    anillo1 = np.c_[r * np.cos(a), np.full(n, y1), r * np.sin(a)]
    v = np.vstack([anillo0, anillo1, [[0, y0, 0], [0, y1, 0]]])
    c0, c1 = 2 * n, 2 * n + 1
    caras = []
    for i in range(n):
        j = (i + 1) % n
        caras += [[i, j, n + j], [i, n + j, n + i], [c0, j, i], [c1, n + i, n + j]]
    return gl.MeshData(vertexes=v, faces=np.array(caras))


def color(hex_, alfa=1.0):
    q = QtGui.QColor(hex_)
    return (q.redF(), q.greenF(), q.blueF(), alfa)


class Vista3D(gl.GLViewWidget):
    UNION_Z, LARGO = 1.0, 0.8   # metros: altura del eje y largo de la pata

    def __init__(self):
        super().__init__()
        self.setBackgroundColor(C["viewport"])
        self.setMinimumHeight(240)
        self.vista("iso")

        g1 = gl.GLGridItem(); g1.setSize(4, 4); g1.setSpacing(0.1, 0.1); g1.setColor((60, 76, 90, 110))
        g2 = gl.GLGridItem(); g2.setSize(4, 4); g2.setSpacing(0.5, 0.5); g2.setColor((90, 110, 128, 170))
        self.addItem(g1); self.addItem(g2)

        alu, alu_osc, anod = color("#d3d9df"), color("#9aa3ac"), color("#3a3f47")
        petg, goma, pcb = color("#d9733a"), color("#1b1d20"), color("#1f5f3c")

        def pieza(malla, col, x, y, z, padre=None, opciones="opaque", sombreado="shaded"):
            m = gl.GLMeshItem(meshdata=malla, color=col, smooth=False, shader=sombreado, glOptions=opciones)
            m.translate(x, y, z)
            if padre is None:
                self.addItem(m)
            else:
                m.setParentItem(padre)
            return m

        Z = self.UNION_Z
        # soporte: base, perfil 40x40, brazo y placa del motor
        pieza(malla_caja(0.36, 0.30, 0.016), alu_osc, 0, 0.29, 0.008)
        pieza(malla_caja(0.04, 0.04, Z + 0.12), alu, 0, 0.24, (Z + 0.12) / 2)
        pieza(malla_caja(0.04, 0.16, 0.04), alu, 0, 0.17, Z + 0.09)
        pieza(malla_caja(0.15, 0.008, 0.17), alu_osc, 0, 0.095, Z + 0.02)
        # motor WK8115-4 y moteus-c1 detras de la placa, cicloidal (PETG) delante
        pieza(malla_cilindro_y(0.048, 0.034), anod, 0, 0.118, Z)
        pieza(malla_cilindro_y(0.040, 0.006), petg, 0, 0.138, Z)
        pieza(malla_caja(0.046, 0.006, 0.046), pcb, 0, 0.144, Z)
        pieza(malla_cilindro_y(0.058, 0.036), petg, 0, 0.072, Z)
        for i in range(8):
            a = i / 8 * 2 * math.pi
            pieza(malla_cilindro_y(0.004, 0.04, 12), anod, math.cos(a) * 0.051, 0.072, Z + math.sin(a) * 0.051)

        # pata: gira con el angulo real (0 = abajo, 90 = horizontal hacia +X)
        self.pivote = gl.GLGraphicsItem.GLGraphicsItem(); self.addItem(self.pivote)
        pieza(malla_cilindro_y(0.038, 0.02), anod, 0, 0.044, 0, self.pivote)
        pieza(malla_cilindro_y(0.03, 0.014), alu, 0, 0.026, 0, self.pivote)
        pieza(malla_caja(0.036, 0.014, self.LARGO), alu, 0, 0.012, -self.LARGO / 2, self.pivote)
        pieza(gl.MeshData.sphere(rows=12, cols=20, radius=0.024), goma, 0, 0.012, -self.LARGO, self.pivote)
        pieza(gl.MeshData.sphere(rows=8, cols=14, radius=0.016), petg, 0, -0.004, -0.29, self.pivote)
        cm = gl.GLTextItem(pos=(0.05, -0.01, -0.29), text="CM", color=QtGui.QColor(C["fg2"]), font=QtGui.QFont("Sans", 9))
        cm.setParentItem(self.pivote)

        # pata fantasma en el destino
        self.fantasma = gl.GLGraphicsItem.GLGraphicsItem(); self.addItem(self.fantasma)
        pieza(malla_caja(0.036, 0.014, self.LARGO), color(C["target"], 0.28), 0, 0.012, -self.LARGO / 2,
              self.fantasma, "translucent", None)
        pieza(gl.MeshData.sphere(rows=8, cols=14, radius=0.024), color(C["target"], 0.28), 0, 0.012, -self.LARGO,
              self.fantasma, "translucent", None)

        # transportador de -90 (colgando) a 90 (vertical arriba); 0 = horizontal
        R = self.LARGO + 0.06
        ang = np.radians(np.arange(0, 181))          # medido desde la vertical hacia abajo
        arco = np.c_[np.sin(ang) * R, np.full(ang.size, 0.03), Z - np.cos(ang) * R]
        self.addItem(gl.GLLinePlotItem(pos=arco, color=color(C["fg2"], 0.7), width=1.5, antialias=True))
        marcas = []
        for d in range(-90, 91, 10):
            a = math.radians(d + 90); largo = 0.05 if d % 30 == 0 else 0.025
            marcas += [[math.sin(a) * R, 0.03, Z - math.cos(a) * R],
                       [math.sin(a) * (R + largo), 0.03, Z - math.cos(a) * (R + largo)]]
            if d % 30 == 0:
                self.addItem(gl.GLTextItem(pos=(math.sin(a) * (R + 0.10), 0.03, Z - math.cos(a) * (R + 0.10)),
                                           text=f"{d}°", color=QtGui.QColor(C["fg"] if d == 0 else C["fg2"]),
                                           font=QtGui.QFont("Sans", 10 if d == 0 else 9)))
        self.addItem(gl.GLLinePlotItem(pos=np.array(marcas), color=color(C["fg2"], 0.7), width=1.5, mode="lines"))
        self.poner(-90, 0, False)

    def vista(self, nombre):
        centro, dist, elev, azim = {
            "iso": ((0.35, 0, 1.0), 5.0, 12, -60),
            "frente": ((0.4, 0, 1.0), 4.6, 2, -90),
            "lado": ((0.0, 0, 1.0), 4.6, 4, 0),
        }[nombre]
        self.opts["center"] = QtGui.QVector3D(*centro)
        self.setCameraPosition(distance=dist, elevation=elev, azimuth=azim)

    def poner(self, ang, destino, mostrar_destino):
        for item, a in ((self.pivote, ang), (self.fantasma, destino)):
            item.resetTransform()
            item.translate(0, 0, self.UNION_Z)
            item.rotate(-(a + 90), 0, 1, 0, local=True)   # 0 = horizontal, 90 = arriba, -90 = colgando
        self.fantasma.setVisible(mostrar_destino)


# ---------------------------------------------------------------- indicador de un par de servos
class Indicador(QtWidgets.QWidget):
    """Semicirculo de -90 a 90 grados (0 = centro, arriba): aguja = comando, punto = encoder, marca = destino."""

    def __init__(self, color_par):
        super().__init__()
        self.col = QtGui.QColor(color_par)
        self.dest = self.cmd = self.enc = float("nan"); self.activo = False
        self.setMinimumSize(170, 104)

    def poner(self, dest, cmd, enc, activo):
        self.dest, self.cmd, self.enc, self.activo = dest, cmd, enc, activo
        self.update()

    def paintEvent(self, _):
        p = QtGui.QPainter(self); p.setRenderHint(QtGui.QPainter.Antialiasing)
        w, h = self.width(), self.height()
        r = max(20.0, min(w / 2 - 24, h - 40)); cx, cy = w / 2, h - 14

        def punto(a, rr):
            rad = math.radians(a + 90)          # angulo con signo -> posicion en el semicirculo
            return QtCore.QPointF(cx - rr * math.cos(rad), cy - rr * math.sin(rad))   # 0 a la izquierda, como la barra

        tenue = QtGui.QColor(C["muted"])
        p.setPen(QtGui.QPen(QtGui.QColor(C["line"]), 6))
        p.drawArc(QtCore.QRectF(cx - r, cy - r, 2 * r, 2 * r), 0, 180 * 16)
        p.setPen(QtGui.QPen(tenue, 1)); f = p.font(); f.setPointSizeF(7.5); p.setFont(f)
        for a in range(-90, 91, 30):
            p.drawLine(punto(a, r + 4), punto(a, r + 9 if a % 90 == 0 else r + 7))
        for a in (-90, 0, 90):
            q = punto(a, r + 17); p.drawText(QtCore.QRectF(q.x() - 16, q.y() - 7, 32, 14), QtCore.Qt.AlignCenter, f"{a:+d}°" if a else "0°")
        if not math.isnan(self.dest) and self.activo:
            p.setPen(QtGui.QPen(QtGui.QColor(C["target"]), 3)); p.drawLine(punto(self.dest, r - 6), punto(self.dest, r + 8))
        if not math.isnan(self.cmd):
            pen = QtGui.QPen(self.col if self.activo else tenue, 3, QtCore.Qt.SolidLine if self.activo else QtCore.Qt.DashLine)
            pen.setCapStyle(QtCore.Qt.RoundCap); p.setPen(pen)
            p.drawLine(QtCore.QPointF(cx, cy), punto(self.cmd, r - 10))
        if not math.isnan(self.enc):
            p.setPen(QtGui.QPen(QtGui.QColor(C["panel"]), 2)); p.setBrush(QtGui.QColor(C["fg"]))
            p.drawEllipse(punto(self.enc, r), 5, 5)
        p.setPen(QtCore.Qt.NoPen); p.setBrush(QtGui.QColor(C["fg2"])); p.drawEllipse(QtCore.QPointF(cx, cy), 4, 4)


# ---------------------------------------------------------------- ventana
QSS = f"""
* {{ font-family: {FUENTE}; font-size: 13px; color: {C['fg']}; }}
QMainWindow, QWidget#raiz, QScrollArea {{ background: {C['bg']}; border: none; }}
QFrame#panel {{ background: {C['panel']}; border: 1px solid {C['line']}; border-radius: 8px; }}
QLabel#ceja {{ color: {C['muted']}; font-size: 11px; font-weight: 600; letter-spacing: 2px; }}
QLabel#titulo {{ font-size: 22px; font-weight: 700; }}
QLabel#sub {{ color: {C['muted']}; font-family: {MONO}; font-size: 12px; }}
QLabel#grande {{ font-size: 46px; font-weight: 700; font-family: {MONO}; }}
QLabel#medio {{ font-size: 24px; font-weight: 700; font-family: {MONO}; }}
QLabel#tenue {{ color: {C['fg2']}; }}
QLabel#dato {{ font-family: {MONO}; }}
QLabel#chip {{ background: {C['panel']}; border: 1px solid {C['line']}; border-radius: 12px; padding: 4px 10px; font-family: {MONO}; }}
QPushButton {{ background: {C['panel2']}; border: 1px solid {C['line']}; border-radius: 6px; padding: 7px 12px; }}
QPushButton:hover {{ border-color: {C['fg2']}; }}
QPushButton:disabled {{ color: {C['muted']}; }}
QPushButton#primario {{ background: {C['accent']}; border-color: {C['accent']}; color: #1a0f08; font-weight: 600; }}
QPushButton#todo {{ background: transparent; border: 2px solid {C['crit']}; color: {C['crit']}; font-weight: 700; padding: 6px 12px; }}
QPushButton#stop {{ background: {C['crit']}; border-color: {C['crit']}; color: white; font-weight: 700; font-size: 15px; padding: 11px; letter-spacing: 1px; }}
QLineEdit, QDoubleSpinBox, QComboBox {{ background: {C['panel2']}; border: 1px solid {C['line']}; border-radius: 6px; padding: 5px 8px; font-family: {MONO}; }}
QComboBox QAbstractItemView {{ background: {C['panel2']}; selection-background-color: {C['line']}; }}
QPlainTextEdit {{ background: {C['panel2']}; border: 1px solid {C['line']}; border-radius: 6px; font-family: {MONO}; font-size: 12px; }}
QSlider::groove:horizontal {{ height: 6px; background: {C['panel2']}; border-radius: 3px; }}
QSlider::sub-page:horizontal {{ background: {C['accent']}; border-radius: 3px; }}
QSlider::handle:horizontal {{ background: {C['fg']}; width: 16px; margin: -6px 0; border-radius: 8px; }}
QProgressBar {{ background: {C['panel2']}; border: 1px solid {C['line']}; border-radius: 3px; height: 10px; }}
"""


def ceja(texto):
    l = QtWidgets.QLabel(texto.upper()); l.setObjectName("ceja"); return l


def panel():
    f = QtWidgets.QFrame(); f.setObjectName("panel")
    lay = QtWidgets.QVBoxLayout(f); lay.setContentsMargins(14, 12, 14, 14); lay.setSpacing(10)
    return f, lay


def barra(color_hex="#45b97c"):
    b = QtWidgets.QProgressBar(); b.setRange(0, 1000); b.setTextVisible(False); b.setFixedHeight(10)
    pintar_barra(b, color_hex); return b


def pintar_barra(b, color_hex):
    css = f"QProgressBar::chunk {{ background: {color_hex}; border-radius: 2px; }}"
    if b.property("css") != css:
        b.setStyleSheet(css); b.setProperty("css", css)


def chip_estilo(lbl, estado):
    borde = {"ok": C["line"], "warn": C["warn"], "crit": C["crit"], "": C["line"]}[estado]
    fondo = "#3a1c1c" if estado == "crit" else C["panel"]
    css = f"border-color: {borde}; background: {fondo};"
    if lbl.property("css") != css:
        lbl.setStyleSheet(css); lbl.setProperty("css", css)


class Ventana(QtWidgets.QMainWindow):
    def __init__(self, puerto_inicial=None):
        super().__init__()
        self.setWindowTitle("Banco de Pata")
        self.resize(1500, 980)
        self.ajustes = QtCore.QSettings("UAO", "BancoPata")
        self.d = {k: float("nan") for k in CAMPOS}
        self.d.update(modo=0, fault=0, banderas=0, can_ok=0, can_env=0, tmax=30, grav=5.7)
        self.ultimo_dato = 0.0; self.cuenta = 0; self.hz = 0
        self.hist = deque(maxlen=int(VENTANA_S * 30))
        self.s = {k: float("nan") for k in CAMPOS_S}
        self.s.update(a_b=0, b_b=0)
        self.hist_s = deque(maxlen=int(VENTANA_S * 30))
        self.puerto = None; self.intentos_p1 = 0
        self.mostrado = -90.0

        self.sim = Simulador(); self.sim.linea.connect(self.al_recibir)
        self._construir()
        self.setStyleSheet(QSS)

        self.t_panel = QtCore.QTimer(self); self.t_panel.timeout.connect(self.refrescar); self.t_panel.start(100)
        self.t_3d = QtCore.QTimer(self); self.t_3d.timeout.connect(self.refrescar_3d); self.t_3d.start(33)
        self.t_graf = QtCore.QTimer(self); self.t_graf.timeout.connect(self.refrescar_graficas); self.t_graf.start(60)
        self.t_hz = QtCore.QTimer(self); self.t_hz.timeout.connect(self._medir_hz); self.t_hz.start(1000)
        self.t_p1 = QtCore.QTimer(self); self.t_p1.timeout.connect(self._pedir_telemetria)

        self.sim.iniciar()
        if puerto_inicial:
            QtCore.QTimer.singleShot(300, lambda: self.conectar(puerto_inicial))

    # ---------------- construccion
    def _construir(self):
        total = QtWidgets.QScrollArea(); total.setWidgetResizable(True); total.setFrameShape(QtWidgets.QFrame.NoFrame)
        raiz = QtWidgets.QWidget(); raiz.setObjectName("raiz"); total.setWidget(raiz); self.setCentralWidget(total)
        principal = QtWidgets.QVBoxLayout(raiz); principal.setContentsMargins(18, 12, 18, 16); principal.setSpacing(12)

        # barra superior
        top = QtWidgets.QHBoxLayout(); top.setSpacing(10)
        t = QtWidgets.QLabel("Banco de Pata"); t.setObjectName("titulo")
        s = QtWidgets.QLabel("WK8115-4 · cicloidal 1:11 · moteus-c1 · ESP32 + MCP2518FD"); s.setObjectName("sub")
        marca = QtWidgets.QVBoxLayout(); marca.setSpacing(0); marca.addWidget(t); marca.addWidget(s)
        top.addLayout(marca); top.addStretch(1)
        self.chip_enlace = self._chip(); self.chip_bat = self._chip(); self.chip_temp = self._chip(); self.chip_falla = self._chip()
        for c in (self.chip_enlace, self.chip_bat, self.chip_temp, self.chip_falla):
            top.addWidget(c)
        self.combo_puerto = QtWidgets.QComboBox(); self.combo_puerto.setMinimumWidth(150)
        b_ref = QtWidgets.QPushButton("↻"); b_ref.setToolTip("Buscar puertos"); b_ref.clicked.connect(self.listar_puertos)
        self.b_conectar = QtWidgets.QPushButton("Conectar ESP32"); self.b_conectar.setObjectName("primario")
        self.b_conectar.clicked.connect(self.alternar_conexion)
        b_todo = QtWidgets.QPushButton("PARAR TODO [x]"); b_todo.setObjectName("todo")
        b_todo.setToolTip("Brushless libre y los cuatro servos sueltos"); b_todo.clicked.connect(lambda: self.enviar("x"))
        top.addWidget(b_todo)
        top.addWidget(self.combo_puerto); top.addWidget(b_ref); top.addWidget(self.b_conectar)
        principal.addLayout(top)
        self.listar_puertos()

        cuerpo = QtWidgets.QHBoxLayout(); cuerpo.setSpacing(12)
        izq = QtWidgets.QVBoxLayout(); izq.setSpacing(12)
        der = QtWidgets.QVBoxLayout(); der.setSpacing(12)
        cuerpo.addLayout(izq, 16)
        cuerpo.addLayout(der, 10)
        principal.addLayout(cuerpo, 1)

        # visor 3D con encabezado
        p3d, l3d = panel()
        cab = QtWidgets.QHBoxLayout()
        col = QtWidgets.QVBoxLayout(); col.setSpacing(0)
        col.addWidget(ceja("Angulo de la pata (0 = horizontal)"))
        self.l_ang = QtWidgets.QLabel("0.0°"); self.l_ang.setObjectName("grande"); col.addWidget(self.l_ang)
        cab.addLayout(col)
        info = QtWidgets.QVBoxLayout(); info.setSpacing(4)
        self.l_dest = QtWidgets.QLabel(); self.l_dest.setObjectName("tenue")
        self.l_err = QtWidgets.QLabel(); self.l_err.setObjectName("tenue")
        self.l_modo = QtWidgets.QLabel(); self.l_modo.setObjectName("tenue")
        for w in (self.l_dest, self.l_err, self.l_modo):
            info.addWidget(w)
        cab.addSpacing(18); cab.addLayout(info); cab.addStretch(1)
        self.l_sim = QtWidgets.QLabel(" SIMULACION "); self.l_sim.setStyleSheet(
            f"background:{C['accent']}; color:#1a0f08; font-weight:700; border-radius:4px; padding:3px 6px;")
        cab.addWidget(self.l_sim, 0, QtCore.Qt.AlignTop)
        for nombre, txt in (("iso", "3/4"), ("frente", "Frente"), ("lado", "Lado")):
            b = QtWidgets.QPushButton(txt); b.clicked.connect(lambda _=False, n=nombre: self.vista.vista(n))
            cab.addWidget(b, 0, QtCore.Qt.AlignTop)
        l3d.addLayout(cab)
        self.vista = Vista3D(); l3d.addWidget(self.vista, 1)
        izq.addWidget(p3d, 5)

        # brazo: dos pares de servos independientes
        fila_servos = QtWidgets.QHBoxLayout(); fila_servos.setSpacing(12)
        self.servo_w = []
        for i, (letra, nombre) in enumerate(PARES):
            fila_servos.addWidget(self._tarjeta_servo(i, letra, nombre, C["arriba"] if letra == "a" else C["abajo"]))
        izq.addLayout(fila_servos)

        # graficas
        pg.setConfigOptions(antialias=True, background=C["panel"], foreground=C["muted"])
        filas = QtWidgets.QHBoxLayout(); filas.setSpacing(12)
        self.graf = []
        for titulo, series, rango in (
            ("Angulo (°)", [("ang", "Real", C["real"], False), ("dest", "Destino", C["target"], True)], (-90, 90)),
            ("Par en la salida (Nm)", [("par", "Medido", C["real"], False), ("ff", "Compensacion", C["target"], True)], (0, 8)),
            ("Servos (°): linea = encoder, rayas = comando",
             [("a_enc", "Arriba", C["arriba"], False), ("a_cmd", None, C["arriba"], True),
              ("b_enc", "Abajo", C["abajo"], False), ("b_cmd", None, C["abajo"], True)], (-90, 90)),
        ):
            pp, lp = panel(); lp.addWidget(ceja(titulo))
            w = pg.PlotWidget(); w.setMinimumHeight(120); w.showGrid(x=False, y=True, alpha=0.25)
            w.setXRange(-VENTANA_S, 0, padding=0); w.setMouseEnabled(x=False, y=False); w.hideButtons()
            w.getAxis("bottom").setLabel("s"); w.setMenuEnabled(False)
            if len(series) > 1:
                w.addLegend(offset=(8, 4), labelTextColor=C["fg2"], colCount=2)
            curvas = []
            for clave, nombre, col, rayas in series:
                pen = pg.mkPen(col, width=2, style=QtCore.Qt.DashLine if rayas else QtCore.Qt.SolidLine)
                curvas.append((clave, w.plot([], [], pen=pen, name=nombre, connect="finite")))
            lp.addWidget(w); filas.addWidget(pp)
            self.graf.append((w, curvas, rango, "s" if curvas[0][0].startswith(("a_", "b_")) else "p"))
        izq.addLayout(filas, 3)

        # mando
        pm, lm = panel()
        fila = QtWidgets.QHBoxLayout(); fila.addWidget(ceja("Mando")); fila.addStretch(1)
        self.f_cero = QtWidgets.QLabel(); self.f_sent = QtWidgets.QLabel(); self.f_ver = QtWidgets.QLabel()
        for f in (self.f_cero, self.f_sent, self.f_ver):
            fila.addWidget(f)
        lm.addLayout(fila)
        fila = QtWidgets.QHBoxLayout()
        self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal); self.slider.setRange(-90, 90); self.slider.setValue(30)
        self.spin = QtWidgets.QDoubleSpinBox(); self.spin.setRange(-90, 90); self.spin.setDecimals(1); self.spin.setSingleStep(0.5)
        self.spin.setValue(30); self.spin.setFixedWidth(80)
        self.slider.valueChanged.connect(lambda v: self.spin.setValue(v))
        self.spin.valueChanged.connect(lambda v: self.slider.setValue(int(round(v))))
        b_ir = QtWidgets.QPushButton("Ir"); b_ir.setObjectName("primario"); b_ir.clicked.connect(lambda: self.ir(self.spin.value()))
        fila.addWidget(self.slider, 1); fila.addWidget(self.spin); fila.addWidget(b_ir)
        lm.addLayout(fila)
        fila = QtWidgets.QHBoxLayout(); fila.setSpacing(6)
        for a in (-90, -45, 0, 30, 45, 60, 90):
            b = QtWidgets.QPushButton(str(a)); b.clicked.connect(lambda _=False, a=a: self.ir(a)); fila.addWidget(b)
        lm.addLayout(fila)
        fila = QtWidgets.QHBoxLayout()
        b_stop = QtWidgets.QPushButton("STOP · MOTOR LIBRE"); b_stop.setObjectName("stop"); b_stop.clicked.connect(lambda: self.enviar("s"))
        nota = QtWidgets.QLabel("Con s la pata cae.\nSostenla antes."); nota.setObjectName("tenue")
        fila.addWidget(b_stop, 1); fila.addWidget(nota)
        lm.addLayout(fila)
        rej = QtWidgets.QGridLayout(); rej.setSpacing(6)
        for i, (txt, cmd, ayuda) in enumerate((("Confirmar cero [v]", "v", "El cero coincide con el transportador"),
                                               ("Cero colgando [z]", "z", "Con la pata colgando quieta (-90), despues de s"),
                                               ("Sentido [h]", "h", "Con la pata horizontal (0), despues de s y z"),
                                               ("Gravedad [m]", "m", "Medir el par de gravedad estando en 0 (horizontal)"),
                                               ("Estado [e]", "e", "Imprimir el estado en la consola"),
                                               ("Calibracion [c]", "c", "Imprimir la calibracion guardada"))):
            b = QtWidgets.QPushButton(txt); b.setToolTip(ayuda)
            b.clicked.connect(lambda _=False, c=cmd: self.enviar(c)); rej.addWidget(b, i // 3, i % 3)
        lm.addLayout(rej)
        fila = QtWidgets.QHBoxLayout()
        lt = QtWidgets.QLabel("Limite de par (Nm)"); lt.setObjectName("tenue")
        self.spin_t = QtWidgets.QDoubleSpinBox(); self.spin_t.setRange(1, 40); self.spin_t.setDecimals(0); self.spin_t.setValue(30); self.spin_t.setFixedWidth(70)
        b_t = QtWidgets.QPushButton("Aplicar"); b_t.clicked.connect(lambda: self.enviar(f"t{self.spin_t.value():.0f}"))
        b_r = QtWidgets.QPushButton("Borrar calibracion"); b_r.clicked.connect(self.borrar)
        fila.addWidget(lt); fila.addWidget(self.spin_t); fila.addWidget(b_t); fila.addStretch(1); fila.addWidget(b_r)
        lm.addLayout(fila)
        der.addWidget(pm)

        # bateria y temperatura
        pe, le = panel()
        fila = QtWidgets.QHBoxLayout(); fila.addWidget(ceja("Bateria y temperatura")); fila.addStretch(1)
        self.combo_celdas = QtWidgets.QComboBox()
        for txt, n in (("Fuente de banco", 0), ("4S LiPo", 4), ("6S LiPo", 6), ("8S LiPo", 8), ("10S LiPo", 10), ("12S LiPo", 12)):
            self.combo_celdas.addItem(txt, n)
        self.combo_celdas.setCurrentIndex(self.combo_celdas.findData(int(self.ajustes.value("celdas", 6))))
        self.combo_celdas.currentIndexChanged.connect(lambda: self.ajustes.setValue("celdas", self.combo_celdas.currentData()))
        fila.addWidget(QtWidgets.QLabel("Bateria")); fila.addWidget(self.combo_celdas)
        le.addLayout(fila)
        fila = QtWidgets.QHBoxLayout()
        self.l_pct = QtWidgets.QLabel("--"); self.l_pct.setObjectName("medio")
        self.l_volt = QtWidgets.QLabel("--"); self.l_volt.setObjectName("dato")
        fila.addWidget(self.l_pct); fila.addStretch(1); fila.addWidget(self.l_volt)
        le.addLayout(fila)
        self.b_bat = barra(); le.addWidget(self.b_bat)
        temps = QtWidgets.QHBoxLayout(); temps.setSpacing(16)
        self.temp_w = {}
        for clave, titulo in (("temp", "Placa moteus"), ("mtemp", "Motor")):
            c = QtWidgets.QVBoxLayout(); c.setSpacing(4)
            c.addWidget(ceja(titulo))
            fila = QtWidgets.QHBoxLayout()
            v = QtWidgets.QLabel("--"); v.setObjectName("medio"); est = QtWidgets.QLabel("--"); est.setObjectName("tenue")
            fila.addWidget(v); fila.addStretch(1); fila.addWidget(est); c.addLayout(fila)
            b = barra(); c.addWidget(b)
            temps.addLayout(c, 1)
            self.temp_w[clave] = (v, est, b)
        le.addLayout(temps)
        fila = QtWidgets.QHBoxLayout(); fila.addWidget(ceja("Sensores de corriente")); fila.addStretch(1)
        self.l_corr = []
        for k in range(3):
            lab = QtWidgets.QLabel(f"I{k + 1} --"); lab.setObjectName("dato"); lab.setToolTip(f"CURRENT{k + 1} del esquematico")
            fila.addWidget(lab); self.l_corr.append(lab)
        b_i = QtWidgets.QPushButton("Cero [i]"); b_i.setToolTip("Poner en cero los sensores (sin carga)")
        b_i.clicked.connect(lambda: self.enviar("i")); fila.addWidget(b_i)
        le.addLayout(fila)
        der.addWidget(pe)

        # datos del moteus
        pd, ld = panel()
        fila = QtWidgets.QHBoxLayout(); fila.addWidget(ceja("Datos del moteus")); fila.addStretch(1)
        self.l_hz = QtWidgets.QLabel("--"); self.l_hz.setObjectName("tenue"); fila.addWidget(self.l_hz)
        ld.addLayout(fila)
        rej = QtWidgets.QGridLayout(); rej.setHorizontalSpacing(18); rej.setVerticalSpacing(5)
        self.kv = {}
        filas_kv = [("ang", "Angulo pata"), ("dest", "Destino"), ("err", "Error"), ("vel", "Velocidad"),
                    ("par", "Par medido"), ("ff", "Par compensacion"), ("iq", "Corriente q"), ("id", "Corriente d"),
                    ("potencia", "Potencia"), ("motor_rev", "Posicion moteus"), ("tmax", "Limite de par"),
                    ("grav", "Par gravedad 0° (horiz.)"), ("modo", "Modo"), ("can", "CAN respuestas")]
        for i, (k, nombre) in enumerate(filas_kv):
            n = QtWidgets.QLabel(nombre); n.setObjectName("tenue")
            v = QtWidgets.QLabel("--"); v.setObjectName("dato"); v.setAlignment(QtCore.Qt.AlignRight)
            rej.addWidget(n, i // 2, (i % 2) * 2); rej.addWidget(v, i // 2, (i % 2) * 2 + 1)
            self.kv[k] = v
        ld.addLayout(rej)
        self.l_falla = QtWidgets.QLabel(); self.l_falla.setWordWrap(True); ld.addWidget(self.l_falla)
        der.addWidget(pd)

        # consola
        pc, lc = panel()
        lc.addWidget(ceja("Consola serie"))
        self.consola = QtWidgets.QPlainTextEdit(); self.consola.setReadOnly(True); self.consola.setMaximumBlockCount(500)
        self.consola.setMinimumHeight(110)
        lc.addWidget(self.consola, 1)
        fila = QtWidgets.QHBoxLayout()
        self.entrada = QtWidgets.QLineEdit(); self.entrada.setPlaceholderText("Comando (ej: 45, a0, a-30, b50, ax, s, x, e)")
        self.entrada.returnPressed.connect(self._enviar_entrada)
        b = QtWidgets.QPushButton("Enviar"); b.clicked.connect(self._enviar_entrada)
        fila.addWidget(self.entrada, 1); fila.addWidget(b); lc.addLayout(fila)
        der.addWidget(pc, 1)

    def _tarjeta_servo(self, i, letra, nombre, col):
        f, l = panel(); l.setSpacing(8)
        fila = QtWidgets.QHBoxLayout()
        muestra = QtWidgets.QLabel(); muestra.setFixedSize(16, 4); muestra.setStyleSheet(f"background:{col}; border-radius:2px;")
        fila.addWidget(muestra); fila.addWidget(ceja(f"Servos {nombre}")); fila.addStretch(1)
        lv = QtWidgets.QLabel("°/s"); lv.setObjectName("tenue")
        vel = QtWidgets.QDoubleSpinBox(); vel.setRange(5, 360); vel.setDecimals(0); vel.setValue(20); vel.setFixedWidth(62)
        vel.setToolTip(f"Velocidad del par (comando {letra}v)")
        b_v = QtWidgets.QPushButton("Vel"); b_v.setToolTip("Aplicar la velocidad")
        b_v.clicked.connect(lambda: self.enviar(f"{letra}v{vel.value():.0f}"))
        fila.addWidget(vel); fila.addWidget(lv); fila.addWidget(b_v); fila.addSpacing(8)
        estado = QtWidgets.QLabel("--"); estado.setMinimumWidth(80); estado.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        fila.addWidget(estado)
        l.addLayout(fila)
        cuerpo = QtWidgets.QHBoxLayout()
        ind = Indicador(col); cuerpo.addWidget(ind, 1)
        rej = QtWidgets.QGridLayout(); rej.setVerticalSpacing(2); rej.setHorizontalSpacing(10)
        vals = {}
        for r, (k, txt) in enumerate((("dest", "Destino"), ("cmd", "Comando"), ("enc", "Encoder"), ("err", "Error"), ("iman", "Iman"))):
            n = QtWidgets.QLabel(txt); n.setObjectName("tenue")
            v = QtWidgets.QLabel("--"); v.setObjectName("dato"); v.setAlignment(QtCore.Qt.AlignRight)
            rej.addWidget(n, r, 0); rej.addWidget(v, r, 1); vals[k] = v
        cuerpo.addLayout(rej)
        l.addLayout(cuerpo)
        fila = QtWidgets.QHBoxLayout()
        sl = QtWidgets.QSlider(QtCore.Qt.Horizontal); sl.setRange(-90, 90); sl.setValue(0)
        sp = QtWidgets.QDoubleSpinBox(); sp.setRange(-90, 90); sp.setDecimals(0); sp.setValue(0); sp.setFixedWidth(64)
        sl.valueChanged.connect(lambda v: sp.setValue(v)); sp.valueChanged.connect(lambda v: sl.setValue(int(v)))
        b_ir = QtWidgets.QPushButton("Ir"); b_ir.setObjectName("primario")
        b_ir.clicked.connect(lambda: self.mover_servos(i, sp.value()))
        fila.addWidget(sl, 1); fila.addWidget(sp); fila.addWidget(b_ir)
        l.addLayout(fila)
        fila = QtWidgets.QHBoxLayout(); fila.setSpacing(4)
        for a in (-90, -45, 0, 45, 90):
            b = QtWidgets.QPushButton(f"{a:+d}" if a else "0"); b.setFixedWidth(48); b.clicked.connect(lambda _=False, a=a: self.mover_servos(i, a))
            fila.addWidget(b)
        fila.addStretch(1)
        b_x = QtWidgets.QPushButton(f"Soltar [{letra}x]"); b_x.setToolTip("Quita la fuerza de los dos servos de este par")
        b_x.clicked.connect(lambda: self.enviar(f"{letra}x"))
        b_z = QtWidgets.QPushButton(f"Alinear [{letra}z]"); b_z.setToolTip("Alinea el encoder con la posicion actual del servo (quieto)")
        b_z.clicked.connect(lambda: self.enviar(f"{letra}z"))
        fila.addWidget(b_x); fila.addWidget(b_z)
        l.addLayout(fila)
        self.servo_w.append(dict(estado=estado, ind=ind, vals=vals, spin=sp))
        return f

    def mover_servos(self, i, a):
        a = max(-90.0, min(90.0, float(a)))
        self.servo_w[i]["spin"].setValue(a)
        self.enviar(f"{PARES[i][0]}{a:g}")

    def refrescar_servos(self):
        s = self.s
        for i, (letra, nombre) in enumerate(PARES):
            w = self.servo_w[i]; b = int(s[f"{letra}_b"]) if not math.isnan(s[f"{letra}_b"]) else 0
            activo, mov = bool(b & 1), bool(b & 64)
            dest, cmd, enc = s[f"{letra}_dest"], s[f"{letra}_cmd"], s[f"{letra}_enc"]
            if not b & 2:
                iman, est_iman = "sin respuesta", "crit"
            elif not b & 4:
                iman, est_iman = "no detectado", "crit"
            elif b & 8:
                iman, est_iman = "debil", "warn"
            elif b & 16:
                iman, est_iman = "fuerte", "warn"
            elif not b & 32:
                iman, est_iman = "OK, sin alinear", "warn"
            else:
                iman, est_iman = "OK", "ok"
            w["vals"]["dest"].setText(fmt(dest, 1, "°") if activo else "--")
            w["vals"]["cmd"].setText(fmt(cmd, 1, "°") if activo else "suelto")
            w["vals"]["enc"].setText(fmt(enc, 1, "°") if b & 32 else f"raw {fmt(s[f'{letra}_raw'], 0)}")
            w["vals"]["err"].setText(fmt(cmd - enc, 1, "°") if activo and b & 32 else "--")
            w["vals"]["iman"].setText(iman)
            w["vals"]["iman"].setStyleSheet(f"color:{C[est_iman]}" if est_iman != "ok" else "")
            texto, col = ("Moviendose", C["target"]) if mov else ("Quietos", C["ok"]) if activo else ("Sueltos", C["muted"])
            w["estado"].setText(texto)
            w["estado"].setStyleSheet(f"color:{col}; font-weight:600;")
            w["ind"].poner(dest, cmd, enc if b & 32 else float("nan"), activo)
        for k, lab in enumerate(self.l_corr):
            lab.setText(f"I{k + 1} {fmt(s[f'i{k + 1}'], 2, 'A')}")

    def _chip(self):
        l = QtWidgets.QLabel("--"); l.setObjectName("chip"); return l

    # ---------------- conexion
    def listar_puertos(self):
        actual = self.combo_puerto.currentText() or self.ajustes.value("puerto", "")
        self.combo_puerto.clear()
        for p in sorted(serial.tools.list_ports.comports(), key=lambda p: ("USB" not in p.device and "ACM" not in p.device, p.device)):
            if p.device.startswith("/dev/ttyS"):  # puertos internos de la placa madre, nunca son el ESP32
                continue
            self.combo_puerto.addItem(p.device)
        if self.combo_puerto.count() == 0:
            self.combo_puerto.addItem("/dev/ttyUSB0")
        i = self.combo_puerto.findText(actual)
        if i >= 0:
            self.combo_puerto.setCurrentIndex(i)

    def alternar_conexion(self):
        if self.puerto:
            self.desconectar()
        else:
            self.conectar(self.combo_puerto.currentText())

    def conectar(self, nombre):
        try:
            self.puerto = PuertoSerie(nombre)
        except (serial.SerialException, OSError) as e:
            self.log(f"No se pudo abrir {nombre}: {e}", "er")
            self.log("Cierra el monitor de PlatformIO y revisa que tu usuario este en el grupo dialout.", "er")
            self.puerto = None
            return
        self.sim.detener(); self.hist.clear(); self.hist_s.clear()
        self.puerto.linea.connect(self.al_recibir)
        self.puerto.perdido.connect(lambda e: self.desconectar(perdido=True))
        self.ajustes.setValue("puerto", nombre)
        self.log(f"Puerto {nombre} abierto a 115200. Esperando al ESP32...", "sys")
        self.intentos_p1 = 0; self.t_p1.start(1800)
        self.b_conectar.setText("Desconectar"); self.l_sim.hide()

    def desconectar(self, perdido=False):
        self.t_p1.stop()
        if self.puerto:
            p = self.puerto; self.puerto = None
            if not perdido:
                p.cerrar()
            else:
                # cerrar igual el descriptor: si queda abierto, el puerto sigue ocupado
                p._activo = False
                try:
                    p.ser.close()
                except Exception:
                    pass
        self.log("Se perdio la conexion con el ESP32." if perdido else "Desconectado. Vuelve la simulacion.", "sys")
        self.hist.clear(); self.hist_s.clear(); self.sim.iniciar()
        self.b_conectar.setText("Conectar ESP32"); self.l_sim.show()

    def _pedir_telemetria(self):
        """El ESP32 se reinicia al abrir el puerto: pedir p1 hasta recibir datos."""
        if time.monotonic() - self.ultimo_dato < 1.5:
            return
        self.intentos_p1 += 1
        if self.intentos_p1 > 12:
            self.t_p1.stop()
            self.log("El ESP32 no envia telemetria. Carga el main.cpp nuevo (comando p1).", "er")
            return
        self.enviar("p1", eco=False)

    def enviar(self, cmd, eco=True):
        if eco:
            self.log("> " + cmd, "tx")
        if self.puerto:
            try:
                self.puerto.enviar(cmd)
            except (serial.SerialException, OSError) as e:
                self.log(f"Error al enviar: {e}", "er")
        else:
            self.sim.enviar(cmd)

    def _enviar_entrada(self):
        t = self.entrada.text().strip()
        if t:
            self.enviar(t); self.entrada.clear()

    def ir(self, a):
        a = max(-90.0, min(90.0, float(a)))
        self.spin.setValue(a)
        self.enviar(f"{a:g}")

    def borrar(self):
        r = QtWidgets.QMessageBox.question(self, "Borrar calibracion",
                                           "Se borran el cero, el sentido y la gravedad guardados en el ESP32. ¿Continuar?")
        if r == QtWidgets.QMessageBox.Yes:
            self.enviar("r")

    # ---------------- datos
    def al_recibir(self, linea):
        if linea.startswith("@T,"):
            f = linea.split(",")
            if len(f) < 21:
                return
            try:
                vals = [float(x) for x in f[1:21]]
            except ValueError:
                return
            self.d.update(zip(CAMPOS, vals))
            for k in ("modo", "fault", "banderas"):
                self.d[k] = int(self.d[k])
            self.ultimo_dato = time.monotonic()
            pos = bool(self.d["banderas"] & 8)
            self.hist.append((self.ultimo_dato, self.d["ang"], self.d["dest"] if pos else float("nan"),
                              self.d["par"], self.d["ff"], self.d["iq"]))
            return
        if linea.startswith("@S,"):
            f = linea.split(",")
            if len(f) < 15:
                return
            try:
                vals = [float(x) for x in f[1:15]]
            except ValueError:
                return
            self.s.update(zip(CAMPOS_S, vals))
            self.ultimo_dato = time.monotonic(); self.cuenta += 1
            s = self.s
            a_on, b_on = int(s["a_b"]) & 1, int(s["b_b"]) & 1
            self.hist_s.append((self.ultimo_dato, s["a_enc"], s["a_cmd"] if a_on else float("nan"),
                                s["b_enc"], s["b_cmd"] if b_on else float("nan")))
            return
        if linea.startswith(">"):
            return
        if linea.strip():
            clase = "wn" if linea.strip().startswith(("ERROR", "ATENCION", "No llego", "Comando no", "Angulo fuera", "Brushless no", "Servos no", "El encoder", "PARAR TODO")) else None
            self.log(linea, clase)

    def log(self, texto, clase=None):
        col = {"tx": C["target"], "wn": C["warn"], "er": C["crit"], "sys": C["muted"]}.get(clase, C["fg"])
        esc = texto.replace("&", "&amp;").replace("<", "&lt;").replace(" ", "&nbsp;")
        barra_v = self.consola.verticalScrollBar(); abajo = barra_v.value() >= barra_v.maximum() - 4
        self.consola.appendHtml(f'<span style="color:{col}">{esc}</span>')
        if abajo:
            barra_v.setValue(barra_v.maximum())

    def _medir_hz(self):
        self.hz, self.cuenta = self.cuenta, 0

    # ---------------- refresco de pantalla
    def refrescar(self):
        d = self.d
        vivo = time.monotonic() - self.ultimo_dato < 1.5
        pos = bool(d["banderas"] & 8)
        if not self.puerto:
            self.chip_enlace.setText("● Simulacion"); chip_estilo(self.chip_enlace, "warn")
        elif vivo:
            self.chip_enlace.setText(f"● ESP32 en vivo · {self.hz} Hz"); chip_estilo(self.chip_enlace, "ok")
        else:
            self.chip_enlace.setText("● Sin datos"); chip_estilo(self.chip_enlace, "crit")
        self.l_hz.setText(f"{self.hz} Hz" if vivo else "sin datos")

        self.refrescar_servos()
        self.l_ang.setText(fmt(d["ang"], 1) + "°")
        self.l_dest.setText("Destino:  " + (fmt(d["dest"], 1) + "°" if pos else "--"))
        self.l_err.setText("Error:  " + (fmt(d["dest"] - d["ang"], 2) + "°" if pos else "--"))
        estado = "Midiendo gravedad" if d["banderas"] & 16 else "En movimiento" if d["banderas"] & 64 else "Sostenida" if pos else "Motor libre"
        self.l_modo.setText(f"Modo {MODOS.get(d['modo'], d['modo'])} · {estado} · Compensacion {fmt(d['ff'], 2)} Nm")

        for lbl, bit, txt in ((self.f_cero, 1, "Cero guardado"), (self.f_sent, 2, "Sentido"), (self.f_ver, 4, "Cero verificado")):
            ok = bool(d["banderas"] & bit)
            lbl.setText(("✓ " if ok else "! ") + txt)
            lbl.setStyleSheet(f"padding:3px 8px; border-radius:4px; background:{'#173226' if ok else '#3a3017'};"
                              f"color:{C['fg']};")

        err = d["dest"] - d["ang"]
        valores = {
            "ang": fmt(d["ang"], 2, "°"), "dest": fmt(d["dest"], 1, "°") if pos else "--",
            "err": fmt(err, 2, "°") if pos else "--", "vel": fmt(d["vel"], 1, "°/s"),
            "par": fmt(d["par"], 2, "Nm"), "ff": fmt(d["ff"], 2, "Nm"), "iq": fmt(d["iq"], 2, "A"),
            "id": fmt(d["id"], 2, "A"), "potencia": fmt(d["potencia"], 1, "W"), "motor_rev": fmt(d["motor_rev"], 4, "rev"),
            "tmax": fmt(d["tmax"], 1, "Nm"), "grav": fmt(d["grav"], 2, "Nm"),
            "modo": f"{d['modo']} {MODOS.get(d['modo'], '')}",
            "can": fmt(100 * d["can_ok"] / d["can_env"], 1, "%") if d["can_env"] else "--",
        }
        for k, v in valores.items():
            self.kv[k].setText(v)
        self.kv["par"].setStyleSheet(f"color:{C['crit']}" if abs(d["par"] or 0) > 0.9 * d["tmax"] else "")

        f = d["fault"]
        if f:
            nombre, ayuda = FALLAS.get(f, (f"Codigo {f}", ""))
            self.l_falla.setText(f"<b>Fault {f}: {nombre}.</b> {ayuda}")
            self.chip_falla.setText(f"● Fault {f} · {nombre}"); chip_estilo(self.chip_falla, "warn" if f >= 96 else "crit")
        else:
            self.l_falla.setText("<b>Sin fallas.</b> El moteus responde normalmente.")
            self.chip_falla.setText("● Sin fallas"); chip_estilo(self.chip_falla, "ok")

        # bateria
        v = d["volt"]; celdas = self.combo_celdas.currentData()
        if celdas and not math.isnan(v):
            vc = v / celdas; pct = lipo_pct(vc)
            col = C["crit"] if pct < 20 else C["warn"] if pct < 40 else C["ok"]
            self.l_pct.setText(f"{pct:.0f} %"); self.l_volt.setText(f"{v:.1f} V · {vc:.2f} V/celda" + ("  (recargar)" if pct < 20 else ""))
            self.b_bat.setValue(int(pct * 10)); pintar_barra(self.b_bat, col)
            self.chip_bat.setText(f"▮ {v:.1f} V · {pct:.0f} %"); chip_estilo(self.chip_bat, "crit" if pct < 20 else "warn" if pct < 40 else "")
        else:
            self.l_pct.setText("--"); self.l_volt.setText(fmt(v, 1, "V") + (" · fuente de banco" if not celdas else ""))
            self.b_bat.setValue(0 if math.isnan(v) else int(min(1000, v / 52 * 1000))); pintar_barra(self.b_bat, C["target"])
            self.chip_bat.setText("▮ " + fmt(v, 1, "V")); chip_estilo(self.chip_bat, "")

        # temperaturas
        for clave, (lv, le, lb) in self.temp_w.items():
            t = d[clave]
            valida = not (math.isnan(t) or t <= -40 or (clave == "mtemp" and t == 0))
            if not valida:
                lv.setText("--"); le.setText("Sin sensor" if clave == "mtemp" else "--"); lb.setValue(0); continue
            est = "crit" if t >= 75 else "warn" if t >= 50 else "ok"
            lv.setText(f"{t:.1f} °C")
            le.setText({"crit": "Falla termica" if clave == "temp" else "Muy caliente",
                        "warn": "Reduce corriente" if clave == "temp" else "Caliente", "ok": "Normal"}[est])
            lb.setValue(int(max(0, min(100, t)) * 10)); pintar_barra(lb, C[est])
        t = d["temp"]
        self.chip_temp.setText("Placa " + fmt(t, 0, "°C"))
        chip_estilo(self.chip_temp, "" if math.isnan(t) else "crit" if t >= 75 else "warn" if t >= 50 else "ok")

    def refrescar_3d(self):
        d = self.d
        if math.isnan(d["ang"]):
            return
        self.mostrado += (d["ang"] - self.mostrado) * 0.35
        pos = bool(d["banderas"] & 8)
        self.vista.poner(self.mostrado, d["dest"] if pos else 0, pos and abs(d["dest"] - d["ang"]) > 0.4)

    def refrescar_graficas(self):
        ahora = time.monotonic()
        fuentes = {
            "p": (self.hist, {"ang": 1, "dest": 2, "par": 3, "ff": 4, "iq": 5}),
            "s": (self.hist_s, {"a_enc": 1, "a_cmd": 2, "b_enc": 3, "b_cmd": 4}),
        }
        for w, curvas, (lo, hi), fuente in self.graf:
            hist, indice = fuentes[fuente]
            if not hist:
                continue
            datos = np.array(hist, dtype=float)
            t = datos[:, 0] - ahora
            mn, mx = lo, hi
            for clave, curva in curvas:
                y = datos[:, indice[clave]]
                curva.setData(t, y)
                ok = y[~np.isnan(y)]
                if ok.size:
                    mn, mx = min(mn, ok.min()), max(mx, ok.max())
            if fuente == "s":
                w.setYRange(-90, 90, padding=0.02)
            else:
                w.setYRange(mn, mx * 1.05 if mx > 0 else mx, padding=0.02)

    def closeEvent(self, e):
        if self.puerto:
            self.puerto.cerrar()
        super().closeEvent(e)


def main():
    app = QtWidgets.QApplication(sys.argv)
    app.setApplicationName("Banco de Pata")
    v = Ventana(sys.argv[1] if len(sys.argv) > 1 else None)
    v.showMaximized()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()