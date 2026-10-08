# Generación de trayectorias: perfiles temporales de la pata

## 1. Propósito

La cinemática inversa dice **qué ángulos** necesita la pierna para poner el
pie en un punto. Falta decidir **cómo se mueve en el tiempo** entre un punto
y otro, para que la velocidad y la aceleración no tengan saltos bruscos.

Este documento describe lo que se implementó, siguiendo la clase
*Manipulator Inverse kinematics II* (control cinemático y generación de
trayectorias, diapositivas 18–74):

- **los métodos de interpolación** de la clase, con sus parámetros;
- **la ley temporal del lápiz**, para que el lápiz recorra cada trazo del
  dibujo con un perfil suave;
- **la pestaña Trayectorias** de la interfaz: gráficas de posición,
  velocidad y aceleración de cada articulación, y un ejecutor que envía la
  consigna a 50 Hz a RViz y, si se quiere, a los motores;
- **el trazador**, donde se dibuja y se elige la ley temporal.

------------------------------------------------------------------------

## 2. Sistema de coordenadas

Todo (archivos de trayectoria, trazador, pestaña Trayectorias) usa el
**mismo sistema que las pestañas de cinemática inversa del equipo**
(`cinematica_directa_der_izq.py`), en mm:

- **+X a lo largo de la pierna, hacia abajo** (vertical). El papel es un
  plano **X = constante**, y **levantar el lápiz es restar a X**.
- **+Y hacia adelante.**
- **Z lateral:** la pierna izquierda está en −Z y la derecha en +Z.

| | Pierna izquierda | Pierna derecha |
|---|---|---|
| Pie colgando (todo a 0°) | (850.28, 0, −306.98) | (850.28, 0, 306.98) |
| Papel recomendado | plano X = 685.28 | plano X = 685.28 |
| Centro del papel (Y, Z) | (0, −489.48) | (0, 489.48) |
| Pose de preparación (20 mm sobre el centro) | (665.28, 0, −489.48) | (665.28, 0, 489.48) |

Por eso el mismo archivo sirve en la pestaña Trayectorias y en "Abrir
archivo…" de las pestañas de cinemática inversa, y `trayectoria.txt` del
equipo (un plano X = 800) también se puede ejecutar en la pestaña
Trayectorias. La elección del papel está en
`ESPACIO_TRABAJO_Y_SUPERFICIE_ESCRITURA.md`.

> Las trayectorias exportadas antes de este cambio (sistema con Z hacia
> arriba, papel en Z = −752.21) ya no sirven: hay que volver a exportarlas
> desde el trazador.

------------------------------------------------------------------------

## 3. Archivos

| Archivo | Contenido |
|---|---|
| `ros2_ws/src/robot_kinematics/robot_kinematics/perfiles_temporales.py` | Los métodos de la clase, la planificación de las 3 articulaciones, la ley temporal del lápiz y la planificación de un dibujo completo |
| `ros2_ws/src/robot_kinematics/robot_kinematics/kinem_v6.py` | Cinemática para las trayectorias: cinemática inversa que respeta los límites del control (ambas piernas), papel recomendado y pose de preparación. Entradas y salidas en el sistema del equipo |
| `ros2_ws/src/robot_kinematics/test/test_perfiles_temporales.py` | Pruebas con los ejemplos resueltos de la clase |
| `ros2_ws/src/robot_kinematics/test/test_kinem_v6.py` | Comprueba que `kinem_v6` da el mismo pie que la cinemática del equipo |
| `ros2_ws/src/robot_teleop/robot_teleop/trayectorias_ui.py` | Pestaña **Trayectorias** de la interfaz |
| `ros2_ws/src/robot_teleop/robot_teleop/teleop_node.py` | Registra la pestaña. El rastro del pie en RViz corta entre trazos |
| `scripts/trazador/trazador.py` | Dibujo a mano alzada, ley temporal del lápiz (simulación) y exportación para la pierna |
| `scripts/trazador/convertir_trayectoria.py` | Convierte un `.txt` en coordenadas del papel al sistema de la pierna |

------------------------------------------------------------------------

## 4. Conceptos

### 4.1 Camino y ley temporal

Un movimiento tiene dos partes que se eligen por separado:

- **El camino:** *por dónde* pasa el pie. En un dibujo, el camino es la
  letra: una sucesión de rectas cortas entre los puntos del archivo.
- **La ley temporal:** *cómo avanza en el tiempo* a lo largo de ese camino.
  Si **s** es la distancia ya recorrida sobre el trazo (de 0 a su longitud
  L), la ley temporal es la función **s(t)**: cuánto ha avanzado el lápiz en
  cada instante.

La posición del lápiz en cada instante es **p(s(t))**: el punto del camino
que está a la distancia s(t) del inicio. El camino no cambia; la ley solo
decide la velocidad con que se recorre (diap. 25–31).

### 4.2 "El lápiz sigue exactamente las rectas del dibujo"

En el modo de dibujo recomendado, la ley temporal se aplica **en el espacio
cartesiano**: cada 20 ms se calcula el punto p(s(t)), que **está sobre las
rectas del dibujo**, y solo después se calcula la cinemática inversa de ese
punto. Por eso el lápiz pasa exactamente por donde se dibujó.

La alternativa es interpolar **los ángulos**: se calculan los ángulos solo
en algunos puntos del dibujo y entre ellos se interpolan los ángulos (no
la posición). Como la relación entre ángulos y posición no es lineal, entre
esos puntos el pie **se curva** y se separa un poco de la recta
(diap. 35–36). Es más suave para los motores, pero el dibujo sale menos
fiel.

### 4.3 Puntos intermedios

Un movimiento **punto a punto** va de A a B: arranca quieto en A y termina
quieto en B.

Un movimiento **por puntos intermedios** (o puntos de paso) va de A a B
**pasando por C, D, …** en tiempos dados, **sin detenerse** en ellos
(diap. 62–73). Hay que decidir la velocidad con que se pasa por cada punto:

- **Cúbico con puntos intermedios:** pasa exactamente por cada punto. Si las
  pendientes de los tramos vecinos cambian de signo, la velocidad en el
  punto es cero; si no, es la media de las dos (diap. 64).
- **Lineal con ajuste parabólico:** tramos de velocidad constante unidos por
  parábolas. Pasa *cerca* de los puntos intermedios, no exactamente, pero
  empieza y termina exactamente en el primero y el último (diap. 66–69).

En la pestaña Trayectorias se usan para **ejecutar un dibujo** interpolando
los ángulos (sección 4.2): los puntos intermedios se toman del archivo cada
5 mm, y sus tiempos se reparten según la velocidad del lápiz.

### 4.4 Por qué la ley temporal no pide más parámetros

Un polinomio de mayor grado tiene más coeficientes, y cada coeficiente
necesita una condición:

| Ley | Grado | Condiciones |
|---|---|---|
| Lineal | 1 | s(0) = 0, s(T) = L |
| Cúbica | 3 | + velocidad inicial y final |
| Quíntica | 5 | + velocidad y aceleración iniciales y finales |

En general esas condiciones pueden tener cualquier valor (por ejemplo, una
articulación que ya viene en movimiento: v0 ≠ 0). En un **trazo del
dibujo**, en cambio, el lápiz **parte quieto y llega quieto** (al final del
trazo o en cada esquina). Entonces:

- la velocidad inicial y final valen 0;
- la aceleración inicial y final valen 0.

Esas condiciones son siempre las mismas y no se piden. Lo único que queda
libre es la **duración T** de cada trazo, y se calcula con la **velocidad
máxima v** y la **aceleración máxima a** del lápiz. El trazador y la
pestaña muestran debajo de la ley elegida qué condiciones usa, y desactivan
v o a cuando esa ley no las usa.

------------------------------------------------------------------------

## 5. Los métodos de la clase

Todos se representan igual: un **polinomio por tramos** (clase `Perfil`)
que se evalúa en posición, velocidad y aceleración. `Perfil.texto()`
devuelve las ecuaciones q(t) en tiempo absoluto, como en la clase.

| # | Método | Diapositivas | Parámetros | Aceleración |
|---|---|---|---|---|
| 1 | **Lineal** | 39–40 | tiempo T | infinita al empezar y al terminar (solo como referencia) |
| 2 | **Cúbico** | 45–49 | T, velocidad inicial v0 y final vf | continua dentro del tramo; salta en los extremos |
| 3 | **Quíntico** | 50–51 | T, v0, vf, aceleración inicial a0 y final af | **sin saltos** (con a0 = af = 0 empieza y acaba en cero) |
| 4 | **Trapezoidal / LSPB** (lineal con ajuste parabólico) | 52–58, 61 | T y **una** de: velocidad máxima, aceleración o tiempo de mezcla t_b | escalones finitos |
| 5 | **Tiempo mínimo** (bang-bang) | 59–60 | aceleración máxima (T resulta = 2·√(Δq/a)) | escalones finitos |
| 6 | **Puntos intermedios** | 62–73 | puntos de paso y sus tiempos; **cúbico** o **lineal con ajuste parabólico** (sección 4.3) | continua en los puntos de paso |

- **Trapezoidal.** Se comprueban los límites de la clase. Con velocidad
  máxima debe cumplirse Δq/T < v ≤ 2Δq/T; con aceleración, a ≥ 4Δq/T²; con
  tiempo de mezcla, 0 < t_b ≤ T/2. Si no se cumplen, se informa el rango
  válido.

### 5.1 Coordinación (diap. 34, 37)

- **Coordinado (isócrono):** las tres articulaciones empiezan y terminan a
  la vez. En el tiempo mínimo cada articulación tendría su propio tiempo:
  se toma el mayor y las demás usan un perfil trapezoidal con la misma
  aceleración máxima.
- **Independiente:** cada articulación termina en su propio tiempo (solo
  cambia algo en el tiempo mínimo).

### 5.2 Validación con los ejemplos de la clase

Las pruebas (`test_perfiles_temporales.py`, 12 pruebas, todas pasan)
reproducen los ejemplos resueltos de la presentación:

| Ejemplo | Resultado de la clase | Implementación |
|---|---|---|
| Cúbico 5° → 25° en 2 s (diap. 47) | q = 5 + 15t² − 5t³ | ✓ |
| Cúbico q(2)=10°, q(4)=60° (diap. 49) | q = −12.5t³ + 112.5t² − 300t + 260 | ✓ |
| Quíntico, mismo caso (diap. 51) | q = 9.375t⁵ − 140.625t⁴ + 812.5t³ − 2250t² + 3000t − 1540 | ✓ |
| Trapezoidal 10° → 60°, 20 °/s, 4 s (diap. 58) | t_b = 1.5 s; q = 10 + 6.67t², −5 + 20t, 60 − 6.67(t−4)² | ✓ |
| Tiempo mínimo 10° → 60°, 13.33 °/s² (diap. 60) | t_f = 3.8735 s | ✓ |
| Parabólico 5° → 25° en 2 s, 40 °/s² (diap. 61) | t_p = 0.293 s, β = 11.71 °/s, q_p = 6.716° | ✓ |
| Cúbico con punto intermedio 5° → 15° → 20° (diap. 65) | c = 25, d = −15; c = 5, d = −5 | ✓ |
| Parabólico por 5°, 20°, 15°, 5° en 0, 2, 3, 4 s, 30 °/s² (diap. 70–73) | t_p = 0.268, 0.435, 0.256, 0.423 s; velocidades 8.04, −5, −12.68 °/s; tramos lineales 1.514, 0.654 s | ✓ |

> **Nota sobre la diapositiva 73.** Para el último tramo lineal el ejemplo
> da 0.660 s, porque usa la fórmula de los tramos *interiores*
> (0.5·t_p4). La fórmula del tramo *final* de la diapositiva 69 (y del libro
> de Craig) usa la mezcla final completa: 1 − 0.423 − 0.5·0.256 = **0.449
> s**. Con 0.660 s la curva no terminaría en reposo en 5°. La
> implementación usa la fórmula de la diapositiva 69, y la prueba lo
> verifica.

------------------------------------------------------------------------

## 6. Ley temporal del lápiz

### 6.1 Las cinco leyes

Para un trazo de longitud L, con velocidad máxima v y aceleración máxima a.
Ejemplo: un trazo de 100 mm con v = 20 mm/s y a = 100 mm/s².

| Ley | Cómo avanza el lápiz | Duración T | Ejemplo |
|---|---|---|---|
| **Lineal** | Velocidad constante v; arranca y frena de golpe (aceleración infinita en los extremos). No usa a | L/v | 5.0 s |
| **Cúbica** | Acelera y frena suave; la aceleración salta al inicio y al final | 1.5·L/v, o más si la aceleración superaría a | 7.5 s |
| **Quíntica** | La más suave: velocidad y aceleración empiezan y terminan en 0 | 1.875·L/v, o más si la aceleración superaría a | 9.4 s |
| **Trapezoidal** | Acelera con a hasta v, sigue a v y frena con a. Si el trazo es corto no llega a v (perfil triangular) | L/v + v/a | 5.2 s |
| **Tiempo mínimo** | Acelera con a la primera mitad y frena con a la segunda. **No usa v** (en 100 mm llega a 100 mm/s) | 2·√(L/a) | 2.0 s |

**v es la velocidad máxima, no la media.** Por eso la quíntica tarda más que
la lineal: su pico es 1.875 veces su velocidad media.

En los trazos **cortos** manda a: un lado de 5 mm con la quíntica no llega
a 20 mm/s (llega a 17.4 mm/s), porque T se alarga para no superar
100 mm/s².

### 6.2 El dibujo completo

Para dibujar se combinan los dos enfoques de la clase (diap. 25–31 y
34–35):

1. **Aproximación:** movimiento **articular** punto a punto desde la pose
   actual hasta encima del primer punto.
2. **Bajar el lápiz** en vertical.
3. **Cada trazo (lápiz abajo):** ley temporal cartesiana (sección 4.2). En
   cada muestra se calcula la cinemática inversa.
4. **Entre trazos (lápiz arriba):** subir en vertical, desplazarse con un
   movimiento **articular** punto a punto y bajar en vertical. Si ese
   movimiento articular bajara a menos de 3 mm del papel, el desplazamiento
   se hace en línea recta.
5. **Al terminar**, levantar el lápiz.

El papel es la altura más baja del archivo (la **X más grande**), y la
altura de levantamiento es la diferencia entre la X más grande y la más
pequeña (20 mm con el trazador).

### 6.3 Esquinas y escalones

Una ley temporal suaviza la velocidad *a lo largo* del trazo, pero en una
**esquina** la dirección cambia de golpe y la aceleración se dispara. Por
eso, por defecto:

- **Se para en las esquinas:** se buscan los giros de más de 30° y cada
  lado entre esquinas tiene su propia ley temporal (el lápiz se detiene en
  la esquina). El giro se mide entre cuerdas de 3 mm, así que la "escalera"
  de una diagonal no cuenta como esquina.
- **Se suavizan los escalones:** las coordenadas redondeadas a milímetros
  dejan las diagonales y curvas en escalera (giros de 45° cada 1 mm). Cada
  lado se suaviza con una media móvil de 3 mm, con los extremos fijos. El
  dibujo se desvía como máximo ~0.5 mm.

Efecto medido sobre `trayectoria.txt` del equipo (pierna izquierda, lápiz a
20 mm/s y 100 mm/s², muestras con el lápiz abajo):

| Ley | Sin esquinas ni suavizado | Con esquinas y suavizado | Duración |
|---|---|---|---|
| Quíntica | aceleración del lápiz 686 mm/s², articular 107 °/s² | **120 mm/s², 26 °/s²** | 46 s |
| Cúbica | 800 mm/s², 99 °/s² | **109 mm/s², 28 °/s²** | 38 s |
| Trapezoidal | 911 mm/s², 100 °/s² | **152 mm/s², 32 °/s²** | 29 s |
| Lineal | 912 mm/s², 136 °/s² | 909 mm/s², 137 °/s² (los saltos son propios del método) | 25 s |

------------------------------------------------------------------------

## 7. El trazador (`scripts/trazador/trazador.py`)

Se dibuja con el ratón; el programa reconoce la figura, interpola puntos
cada 1 mm y levanta el lápiz entre figuras.

**Papel en la pierna.** Dónde queda el dibujo en el sistema de la sección
2: X del papel, centro (Y, Z), levantamiento del lápiz, escala y espejo. El
botón **"Valores recomendados"** pone los de la pierna elegida (y el área
de 255 × 255 mm). El dibujo se ve como lo vería alguien de pie **detrás del
robot**: arriba del dibujo = adelante (+Y), derecha del dibujo = derecha
del robot (+Z). Con "Comprobar alcance", los puntos que la pierna no
alcanza dentro de los límites del control se marcan en rojo.

**Ley temporal del lápiz.**

- **Método** (sección 6.1). Debajo se muestra qué condiciones usa, y los
  campos que no usa quedan desactivados.
- **v máx [mm/s]** y **a máx [mm/s²]**.
- **Parar en esquinas y suavizar escalones** (sección 6.3; en el trazador
  van juntas y el umbral es 30°).

Con la ley temporal, el trazador hace dos cosas:

1. **"Simular trayectoria"** anima el lápiz sobre el lienzo al ritmo real
   (50 Hz) y muestra el tiempo total y el tiempo con el lápiz abajo. Con el
   lápiz arriba la simulación es aproximada: el trazador sigue la recta,
   mientras que la interfaz hace un movimiento articular.
2. **"Exportar para la pierna"** guarda los puntos en el sistema de la
   pierna y anota la ley en el encabezado, por ejemplo
   `# ley_temporal metodo=quintico v=20 a=100 esquinas=si`. **El archivo
   solo guarda los puntos**; los tiempos los calcula la interfaz. Al abrir
   el archivo en la pestaña Trayectorias, su sección 3 se rellena con esos
   valores.

------------------------------------------------------------------------

## 8. La pestaña "Trayectorias" de la interfaz

La pestaña ejecuta **solo trayectorias dibujadas en el trazador**. Los
movimientos punto a punto a una coordenada o a unos ángulos cualesquiera se
hacen desde las pestañas de cinemática directa e inversa.

### 8.1 Sección 1: Pose de la pierna

- **Actual:** posición del pie (X, Y, Z) y ángulos (roll, pitch, rodilla)
  de la pose actual. Se actualiza sola, también cuando la pierna se mueve
  desde otras pestañas.
- **Destino:** lo mismo para el destino del último botón pulsado.
- **Pose de preparación:** lleva la pierna a 20 mm sobre el centro del
  papel recomendado.
- **Home:** lleva la pierna colgando (todos los ángulos en 0).

Los dos botones **mueven la pierna directamente**, con un movimiento
articular suave (el perfil de la sección 3): calculan el movimiento, lo
grafican y lo ejecutan. Con la velocidad articular por defecto (15 °/s),
ir de Home a la pose de preparación tarda 8.4 s. No borran el rastro del
último dibujo en RViz.

### 8.2 Sección 2: Dibujo

Un `.txt` del trazador (o cualquier archivo de puntos `x, y, z` en el
sistema de la sección 2).

- **Archivo…:** elige el archivo y carga la ley temporal de su encabezado
  en la sección 4.
- **Modo:**
  - **Ley temporal cartesiana + articular** (recomendado): sección 6.2; el
    lápiz sigue exactamente las rectas del dibujo.
  - **Puntos intermedios articulares (cúbico / parabólico):** interpola
    los ángulos por puntos tomados cada 5 mm (secciones 4.2 y 4.3). Más
    suave para los motores, pero el pie se aparta un poco de las rectas.
    Aquí v es la velocidad media del lápiz. El parabólico usa la a máx de
    la sección 3; si las mezclas no caben, la pestaña pide aumentarla (con
    un cuadrado de 200 mm hace falta al menos 100 °/s²).

### 8.3 Sección 3: Movimientos articulares

El perfil de los movimientos en los que se interpolan **los ángulos**: la
aproximación al primer punto del dibujo, los desplazamientos con el lápiz
levantado y los botones Home y Pose de preparación.

- **Método:** lineal, cúbico, quíntico, trapezoidal o tiempo mínimo
  (sección 5). Empiezan y terminan en reposo.
- **v art. [°/s]:** velocidad articular máxima; con ella se calcula la
  duración de cada movimiento. Por defecto 15 °/s, lo que permite el
  firmware (sección 9). El trapezoidal usa un tiempo de mezcla de T/3.
- **a máx [°/s²]:** solo la usan el tiempo mínimo (que no usa v art.) y el
  modo de dibujo parabólico; si no, queda desactivada.
- **Coordinado (isócrono):** sección 5.1.

### 8.4 Sección 4: Ley temporal del lápiz

Método, v máx, a máx, parar en esquinas (con el umbral en grados) y
suavizar escalones (secciones 6.1 y 6.3). Debajo se muestra qué condiciones
usa la ley elegida. Con los modos de puntos intermedios solo cuenta v.

### 8.5 Botones

- **Calcular y graficar:** arma el plan del dibujo sin mover nada.
- **Ejecutar dibujo:** recalcula y envía los ángulos a ~50 Hz: RViz se
  mueve, y los sliders y el rastro se actualizan. La posición se calcula
  con el reloj real, así que un retraso de la interfaz no deforma el perfil.
- **Parar:** detiene el movimiento donde esté (también el de Home o
  preparación).
- **+ motores:** además de RViz, envía cada consigna a los servos
  (`/servo_commands`), también en los botones Home y Pose de preparación.
  **Prueba siempre primero sin marcarlo.**

### 8.6 Resultados (lado derecho)

- **Línea de estado:**
  - ✓ listo para ejecutar;
  - ✗ algún ángulo se sale de los límites del control, o se supera la
    velocidad del servo: **0.21 s/60° a 12 V ≈ 285 °/s** sin carga
    (`docs/Servo-Motor-RDS51150-12V.pdf`);
  - ⚠ se supera la velocidad que permite el firmware (sección 9), o la
    pierna empieza **por debajo** del papel (sección 10).
- **Cuadro de texto:** duración, número de muestras, velocidad y
  aceleración máximas por articulación, y la lista de tramos con sus
  tiempos (aproximación, trazo 1 lado 1/4, subir el lápiz…).
- **Gráficas:** posición, velocidad y aceleración de roll (azul), pitch
  (rojo) y rodilla (verde). La **banda gris** marca el lápiz abajo y la
  **línea vertical** avanza mientras se ejecuta. En la aceleración se ve la
  diferencia entre métodos: saltos con lineal y cúbico, curvas suaves con
  quíntico y "parar en esquinas".

------------------------------------------------------------------------

## 9. Limitación del firmware

El firmware del ESP32 (`firmware/esp32_servos/src/main.cpp`) no aplica el
ángulo recibido directamente: **avanza 0.3° por actualización** hacia él, y
con mensajes cada 20 ms actualiza aproximadamente una vez por mensaje. Eso
limita a unos **15 °/s** por articulación. Es una estimación: el número
exacto depende del ritmo de mensajes y del bucle del ESP32.

- **Al dibujar** las velocidades articulares son de unos 5 °/s, así que el
  servo sigue el perfil.
- **Los movimientos rápidos** se harían más lentos en el robot real que en
  RViz. Por eso los movimientos articulares de la pestaña (sección 8.3)
  calculan su duración con una velocidad articular de 15 °/s: ir de Home a
  la pose de preparación con el quíntico tarda 8.4 s (a 2 s llegaría a
  63 °/s).

Para que el robot real siga exactamente el perfil habría que cambiar el
firmware para que escriba el ángulo recibido sin la rampa de 0.3°. No se
modificó: es una decisión del equipo.

------------------------------------------------------------------------

## 10. Pose de preparación y montaje

El papel recomendado está **165 mm por encima del pie colgando** (plano
X = 685.28). **Con la mesa puesta la pierna no puede colgar.** El
procedimiento es:

1. Sin la mesa, en la pestaña Trayectorias: botón **Pose de
   preparación**. La pierna va sola (8.4 s con 15 °/s).
2. Colocar la mesa con el papel.
3. **Archivo…** → elegir el archivo → Calcular y graficar → **Ejecutar
   dibujo**. El dibujo empieza y termina con el lápiz levantado.
4. Al terminar, retirar la mesa antes de volver a Home o apagar los
   servos.

------------------------------------------------------------------------

## 11. Pruebas realizadas

- **Perfiles:** las 12 pruebas con los ejemplos de la clase pasan.
- **Cinemática:** `test_kinem_v6.py` (6 pruebas) comprueba, en ambas
  piernas, que `kinem_v6` da el mismo pie que `cinematica_directa_der_izq`,
  que la cinemática inversa vuelve al mismo punto, y que todo el papel
  recomendado es alcanzable con la rodilla en flexión.
- **Interfaz real (`teleop_node`), pierna izquierda:**
  - botón Pose de preparación: la pierna se mueve sola y el pie llega
    exactamente a (665.28, 0, −489.48); botón Home: vuelve a la pierna
    colgando sin borrar el rastro;
  - un cuadrado de 200 mm exportado por el trazador: el lápiz queda
    exactamente en el plano del papel y nunca baja de él;
  - `trayectoria.txt` del equipo (plano X = 800) también se planifica sin
    errores, con la ley cartesiana y con los dos modos de puntos
    intermedios;
  - la ley temporal del encabezado del archivo se carga en la sección 3, y
    los campos v y a se desactivan según la ley.
- **Trazador:** abre, cambia de pierna con los valores recomendados y
  muestra las condiciones de cada ley.
- **No probado:** el robot real y la interacción manual con los botones.
