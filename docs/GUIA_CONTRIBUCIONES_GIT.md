# Guía de contribución al repositorio --- Robot Bípedo

## 1. Propósito de este documento

Este documento establece el procedimiento que deberán seguir todas las
personas que participen en el desarrollo de software del proyecto
**Robot Bípedo**.

El objetivo es mantener el repositorio organizado, evitar que los
cambios de un integrante interfieran con los de otro y garantizar que
todo código incorporado a la rama principal haya sido revisado y
aprobado.

La regla principal del proyecto es:

> **Ningún integrante debe realizar cambios directamente sobre `main`.
> Todo desarrollo debe realizarse en una rama propia y debe incorporarse
> mediante un Pull Request después de la revisión y aprobación
> correspondiente.**

------------------------------------------------------------------------

# 2. Estructura general del trabajo

El repositorio utiliza Git para el control de versiones y GitHub como
repositorio remoto.

De manera simplificada, el flujo será:

``` text
                    GitHub
                       │
                     main
                       │
              ┌────────┴────────┐
              │                 │
        rama de A          rama de B
              │                 │
          desarrollo         desarrollo
              │                 │
              └────────┬────────┘
                       │
                 Pull Request
                       │
                       ▼
                 Revisión del
                    líder
                       │
              ┌────────┴────────┐
              │                 │
          Correcciones        Aprobado
              │                 │
              └──► revisión     ▼
                           Merge → main
```

La rama `main` representa la versión principal e integrada del proyecto.

------------------------------------------------------------------------

# 3. Antes de comenzar

Cada integrante debe tener instalado y configurado:

-   Git.
-   Una cuenta de GitHub con acceso al repositorio.
-   VS Code u otro editor de código.
-   Las herramientas correspondientes a su área de trabajo, por ejemplo:
    -   ROS 2.
    -   Python.
    -   MATLAB/Simulink.
    -   Gazebo.
    -   C/C++.
    -   Micro-ROS, cuando corresponda.

El acceso al repositorio será proporcionado por el administrador/líder
del proyecto mediante GitHub.

------------------------------------------------------------------------

# 4. Clonar el repositorio por primera vez

El repositorio debe clonarse **una sola vez por computador**.

Abrir una terminal y dirigirse a la ubicación donde se desea almacenar
el proyecto.

Por ejemplo:

``` bash
cd ~/Documentos
```

Luego clonar el repositorio.

### Usando SSH

``` bash
git clone git@github.com:JuanMiguelC27/Robot_Bipedo.git
```

### Usando HTTPS

``` bash
git clone https://github.com/JuanMiguelC27/Robot_Bipedo.git
```

> Se recomienda utilizar SSH si el computador ya tiene una clave SSH
> configurada para GitHub.

Después de clonar:

``` bash
cd Robot_bipedoCode
```

La estructura local será similar a:

``` text
Robot_bipedoCode/
├── README.md
├── .gitignore
├── Robot_bipedoCode.code-workspace
├── ros2_ws/
├── matlab/
├── scripts/
├── tests/
└── docs/
```

Para comprobar que el repositorio está correctamente conectado:

``` bash
git remote -v
```

Deberá aparecer la dirección del repositorio remoto.

------------------------------------------------------------------------

# 5. No volver a utilizar `git clone`

`git clone` se utiliza únicamente cuando se obtiene el repositorio por
primera vez.

Una vez que el repositorio ya está en el computador, para obtener
cambios nuevos se utilizará:

``` bash
git pull
```

Por ejemplo, **no se debe volver a clonar el repositorio cada vez que se
quiera trabajar**.

------------------------------------------------------------------------

# 6. Antes de comenzar cualquier trabajo

Antes de crear código, modificar archivos o iniciar una nueva tarea,
siempre se debe actualizar la rama `main`.

Desde la carpeta raíz del repositorio:

``` bash
cd Robot_bipedoCode
```

Comprobar la rama actual:

``` bash
git branch
```

Cambiar a `main`:

``` bash
git switch main
```

Actualizar el repositorio local:

``` bash
git pull
```

El procedimiento inicial siempre debe ser:

``` bash
git switch main
git pull
```

Esto garantiza que la nueva tarea comience a partir de la versión más
reciente del proyecto.

------------------------------------------------------------------------

# 7. Crear una rama para la tarea

**Nunca se debe desarrollar directamente sobre `main`.**

Después de actualizar `main`, se debe crear una rama específica para la
tarea que se va a desarrollar.

Por ejemplo, si se va a implementar un controlador PID:

``` bash
git switch -c feature/control-pid
```

Esto crea una nueva rama llamada:

``` text
feature/control-pid
```

y cambia automáticamente a ella.

Comprobar la rama:

``` bash
git branch
```

La rama actual aparecerá marcada con `*`:

``` text
* feature/control-pid
  main
```

A partir de este momento, todos los cambios realizados pertenecerán a la
rama de la tarea.

------------------------------------------------------------------------

# 8. Convención para nombres de ramas

Las ramas deben tener nombres claros y descriptivos.

Se recomienda utilizar las siguientes categorías:

  --------------------------------------------------------------------------
  Prefijo                 Uso                     Ejemplo
  ----------------------- ----------------------- --------------------------
  `feature/`              Nueva funcionalidad     `feature/control-pid`

  `fix/`                  Corrección de errores   `fix/imu-calibration`

  `refactor/`             Reorganización o mejora `refactor/control-node`
                          interna                 

  `docs/`                 Documentación           `docs/ros2-architecture`

  `test/`                 Pruebas                 `test/kinematics`
  --------------------------------------------------------------------------

Ejemplos para el proyecto:

``` text
feature/robot-urdf
feature/inverse-kinematics
feature/gazebo-simulation
feature/imu-node
feature/teleoperation
feature/motor-control

fix/joint-limit
fix/imu-frame

docs/control-architecture

test/kinematics
```

Evitar nombres poco descriptivos como:

``` text
rama1
prueba
codigo-nuevo
final
final2
cosas
juan
```

------------------------------------------------------------------------

# 9. Trabajar en la rama

Una vez creada la rama, se puede comenzar el desarrollo.

Por ejemplo:

``` text
Robot_bipedoCode/
└── matlab/
    └── control/
        └── pid_controller.m
```

o:

``` text
Robot_bipedoCode/
└── ros2_ws/
    └── src/
        └── robot_control/
            └── robot_control/
                └── controller_node.py
```

Cada integrante debe trabajar principalmente sobre los archivos
relacionados con su tarea.

No modificar archivos pertenecientes a otra funcionalidad sin
coordinación con la persona responsable.

------------------------------------------------------------------------

# 10. Comprobar los cambios

Antes de guardar los cambios en Git, comprobar qué archivos fueron
modificados:

``` bash
git status
```

Por ejemplo:

``` text
modified:   matlab/control/pid_controller.m
```

También se puede revisar exactamente qué cambió:

``` bash
git diff
```

Es recomendable revisar los cambios antes de realizar el commit.

En el caso de ROS 2, también se deben realizar las pruebas y
compilaciones necesarias antes de solicitar la integración.

Por ejemplo:

``` bash
cd ros2_ws
colcon build
```

Si el proyecto requiere ejecutar pruebas:

``` bash
colcon test
```

No se debe solicitar la integración de código que no haya sido
comprobado.

------------------------------------------------------------------------

# 11. Preparar los cambios para el commit

Cuando el trabajo esté listo:

``` bash
git add .
```

Esto prepara los cambios para el siguiente commit.

Antes de continuar, comprobar:

``` bash
git status
```

Si se desea agregar únicamente archivos específicos, también se puede
hacer:

``` bash
git add ruta/al/archivo.py
```

Por ejemplo:

``` bash
git add matlab/control/pid_controller.m
```

------------------------------------------------------------------------

# 12. Convención para los commits

Los mensajes de commit deben ser breves, claros y describir la acción
realizada.

Se recomienda utilizar:

``` text
tipo: descripción
```

Tipos principales:

  -----------------------------------------------------------------------
  Tipo                                Uso
  ----------------------------------- -----------------------------------
  `feat`                              Nueva funcionalidad

  `fix`                               Corrección de errores

  `refactor`                          Reorganización o mejora del código
                                      sin cambiar su funcionalidad

  `docs`                              Documentación

  `test`                              Pruebas

  `chore`                             Tareas de configuración o
                                      mantenimiento
  -----------------------------------------------------------------------

Ejemplos:

``` bash
git commit -m "feat: implement PID controller"
```

``` bash
git commit -m "feat: add inverse kinematics solver"
```

``` bash
git commit -m "fix: correct IMU frame transformation"
```

``` bash
git commit -m "docs: update ROS 2 architecture"
```

``` bash
git commit -m "test: add kinematics validation"
```

Evitar mensajes como:

``` text
git commit -m "cambios"
git commit -m "final"
git commit -m "arreglado"
git commit -m "prueba"
```

El mensaje debe permitir entender qué se modificó sin tener que abrir el
código.

------------------------------------------------------------------------

# 13. Subir la rama a GitHub

Después del commit, la rama debe subirse al repositorio remoto.

La primera vez:

``` bash
git push -u origin feature/control-pid
```

En adelante, mientras se continúe trabajando en la misma rama:

``` bash
git push
```

Por ejemplo:

``` bash
git add .
git commit -m "feat: implement PID controller"
git push -u origin feature/control-pid
```

La rama aparecerá ahora en GitHub.

------------------------------------------------------------------------

# 14. Crear el Pull Request

Después de hacer `push`, se debe crear un **Pull Request (PR)** en
GitHub.

El Pull Request debe solicitar la integración:

``` text
feature/control-pid
        ↓
      main
```

El objetivo del Pull Request es solicitar formalmente que el código sea
revisado antes de incorporarlo a `main`.

El Pull Request debe incluir:

-   Un título claro.
-   Una descripción breve de lo realizado.
-   Los principales archivos o componentes modificados.
-   Las pruebas realizadas.
-   Cualquier consideración importante para la integración.
-   El responsable/revisor correspondiente.

Ejemplo de título:

``` text
feat: implement PID controller
```

Ejemplo de descripción:

``` text
## Descripción

Se implementó el controlador PID para el control de posición.

## Cambios principales

- Implementación del controlador.
- Configuración de Kp, Ki y Kd.
- Pruebas de respuesta temporal.

## Pruebas

- Simulación ejecutada correctamente.
- Validación de respuesta del controlador.

## Revisión

Solicito revisión de la implementación y de los parámetros utilizados.
```

------------------------------------------------------------------------

# 15. Esperar la revisión

Una vez creado el Pull Request, **no se debe hacer merge directamente**.

El líder/revisor analizará:

-   Calidad del código.
-   Arquitectura.
-   Funcionamiento.
-   Compatibilidad con el resto del proyecto.
-   Convenciones utilizadas.
-   Posibles errores.
-   Pruebas realizadas.
-   Documentación necesaria.

El Pull Request puede terminar en dos estados principales:

### A. Aprobado

Si el código está correctamente implementado, el líder aprobará el Pull
Request y se podrá realizar el merge hacia `main`.

``` text
feature/control-pid
        ↓
   Pull Request
        ↓
     APPROVED
        ↓
      main
```

### B. Se solicitan cambios

Si se detectan problemas, el líder puede solicitar correcciones.

En ese caso, **no se debe crear otro Pull Request para la misma tarea**.

Se debe continuar trabajando sobre la misma rama.

------------------------------------------------------------------------

# 16. ¿Qué hacer si solicitan correcciones?

Supongamos que el Pull Request recibe:

``` text
Request changes
```

con una observación como:

> "Separar la lógica del controlador de la comunicación ROS 2."

La persona vuelve a su rama:

``` bash
git switch feature/control-pid
```

Realiza las modificaciones solicitadas.

Después:

``` bash
git status
```

Revisa los cambios:

``` bash
git diff
```

Los agrega:

``` bash
git add .
```

Crea un nuevo commit:

``` bash
git commit -m "refactor: separate controller logic from ROS 2 node"
```

Y realiza:

``` bash
git push
```

El Pull Request se actualizará automáticamente.

No es necesario crear un nuevo Pull Request.

El líder podrá revisar nuevamente los cambios.

------------------------------------------------------------------------

# 17. Mantener la rama actualizada mientras se trabaja

Si la tarea tarda varios días, `main` puede recibir cambios de otros
integrantes.

Por ejemplo:

``` text
Día 1:
main
  └── versión inicial

Día 2:
otro integrante integra cambios
  └── main actualizado

Tu rama:
feature/control-pid
  └── versión anterior
```

Antes de continuar trabajando o antes de solicitar la integración, es
recomendable actualizar la rama con los cambios recientes de `main`.

Una forma sencilla es:

``` bash
git switch main
git pull
git switch feature/control-pid
git merge main
```

Si aparecen conflictos, deberán resolverse antes de continuar.

> Si el equipo establece posteriormente una estrategia diferente, como
> `rebase`, se utilizará el procedimiento definido por el líder. No
> realizar `rebase` por iniciativa propia sobre ramas compartidas.

------------------------------------------------------------------------

# 18. ¿Qué pasa si otra persona subió cambios?

Git no funciona como un sistema donde el último usuario sobrescribe
completamente el trabajo anterior.

Por ejemplo, si inicialmente existe:

``` text
ros2_ws/src/
```

y dos personas crean:

``` text
Persona A:
ros2_ws/src/robot_control/

Persona B:
ros2_ws/src/robot_simulation/
```

Git puede integrar ambos cambios:

``` text
ros2_ws/src/
├── robot_control/
└── robot_simulation/
```

Sin embargo, si dos personas modifican exactamente la misma parte de un
archivo, Git puede detectar un conflicto.

------------------------------------------------------------------------

# 19. Resolución de conflictos

Un conflicto puede aparecer cuando dos ramas modifican la misma parte de
un archivo de maneras incompatibles.

Git puede mostrar algo como:

``` text
<<<<<<< HEAD
kp = 20
=======
kp = 15
>>>>>>> main
```

Esto significa que Git necesita que el desarrollador determine qué
versión debe conservarse.

El conflicto debe resolverse cuidadosamente.

Después de resolverlo:

``` bash
git add .
```

y realizar el commit correspondiente:

``` bash
git commit -m "fix: resolve merge conflict"
```

Después:

``` bash
git push
```

Si existe un Pull Request abierto, este se actualizará automáticamente.

> Si no se comprende un conflicto, no se debe resolver de manera
> arbitraria. Consultar al responsable del área o al líder antes de
> eliminar o reemplazar código de otro integrante.

------------------------------------------------------------------------

# 20. Después de que el Pull Request sea integrado

Una vez que el Pull Request haya sido aprobado y fusionado con `main`,
la tarea se considera integrada.

Antes de comenzar una nueva tarea, volver a actualizar `main`:

``` bash
git switch main
git pull
```

Después crear una nueva rama:

``` bash
git switch -c feature/nueva-tarea
```

No reutilizar una rama antigua para una tarea completamente diferente.

------------------------------------------------------------------------

# 21. Ejemplo completo

Supongamos que una persona de Control debe implementar un controlador
PID.

### Paso 1 --- Entrar al repositorio

``` bash
cd ~/Documentos/Robot_bipedoCode
```

### Paso 2 --- Actualizar `main`

``` bash
git switch main
git pull
```

### Paso 3 --- Crear la rama

``` bash
git switch -c feature/control-pid
```

### Paso 4 --- Desarrollar

Crear o modificar:

``` text
matlab/control/pid_controller.m
```

### Paso 5 --- Comprobar

``` bash
git status
git diff
```

Realizar las pruebas correspondientes.

### Paso 6 --- Preparar cambios

``` bash
git add .
```

### Paso 7 --- Crear commit

``` bash
git commit -m "feat: implement PID controller"
```

### Paso 8 --- Subir la rama

``` bash
git push -u origin feature/control-pid
```

### Paso 9 --- Crear Pull Request

En GitHub:

``` text
feature/control-pid → main
```

Solicitar la revisión del líder.

### Paso 10 --- Si se solicitan cambios

Modificar el código y ejecutar:

``` bash
git add .
git commit -m "refactor: improve PID implementation"
git push
```

El Pull Request se actualizará.

### Paso 11 --- Si se aprueba

El líder realizará el merge.

### Paso 12 --- Preparar la siguiente tarea

``` bash
git switch main
git pull
git switch -c feature/nueva-tarea
```

------------------------------------------------------------------------

# 22. Reglas importantes del proyecto

Todos los integrantes deben cumplir las siguientes reglas:

1.  **No hacer `push` directamente a `main`.**
2.  **No desarrollar directamente sobre `main`.**
3.  Crear una rama propia para cada tarea.
4.  Actualizar `main` antes de crear una nueva rama.
5.  Utilizar nombres descriptivos para las ramas.
6.  Utilizar mensajes de commit claros.
7.  Probar el código antes de crear el Pull Request.
8.  Crear un Pull Request para solicitar la integración.
9.  Esperar la revisión y aprobación correspondiente.
10. Si se solicitan cambios, corregirlos en la misma rama y volver a
    hacer `push`.
11. No eliminar ni modificar código de otra área sin coordinación.
12. No subir archivos generados automáticamente por ROS 2.
13. No subir contraseñas, tokens, claves privadas ni información
    sensible.
14. Consultar al líder o responsable del área cuando exista un conflicto
    o una duda sobre la arquitectura.
15. Mantener la documentación actualizada cuando una modificación afecte
    el funcionamiento o arquitectura del sistema.

------------------------------------------------------------------------

# 23. Flujo resumido

Para recordar el procedimiento:

``` bash
# 1. Primera vez solamente
git clone git@github.com:USUARIO/Robot_bipedoCode.git
cd Robot_bipedoCode

# 2. Antes de cada nueva tarea
git switch main
git pull

# 3. Crear rama
git switch -c feature/nombre-de-la-tarea

# 4. Trabajar y probar el código

# 5. Revisar cambios
git status
git diff

# 6. Preparar cambios
git add .

# 7. Crear commit
git commit -m "tipo: descripción"

# 8. Subir rama
git push -u origin feature/nombre-de-la-tarea

# 9. Crear Pull Request en GitHub
#    feature/nombre-de-la-tarea → main

# 10. Si se solicitan cambios:
#     modificar código → git add . → git commit → git push

# 11. Después del merge
git switch main
git pull
```

------------------------------------------------------------------------

# 24. Principio fundamental

El objetivo de este flujo no es agregar burocracia innecesaria al
desarrollo.

El objetivo es garantizar que:

``` text
Desarrollo individual
        ↓
      Rama
        ↓
     Commit
        ↓
      Push
        ↓
 Pull Request
        ↓
 Revisión técnica
        ↓
   Correcciones
        ↓
    Aprobación
        ↓
      main
```

De esta manera, `main` se mantiene como una versión estable y controlada
del proyecto, mientras cada integrante puede desarrollar sus
funcionalidades de manera independiente sin sobrescribir accidentalmente
el trabajo de los demás.
