# Estado medido que el control publica para el resto del sistema.
# name: nombres de las juntas (DEBEN coincidir con el URDF, ej. Hip_Joint, Knee_Joint,
# Ankle_Joint). Sin esto, robot_state_publisher no puede emparejar angulo->junta y
# RViz dice "No transform".
string[] name
float64[] position
float64[] velocity
float64[] effort
float64 battery
