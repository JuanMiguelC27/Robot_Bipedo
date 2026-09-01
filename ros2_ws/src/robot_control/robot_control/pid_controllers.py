# PID por junta: corrige el error entre lo que quieres y lo que mides.
# Guarda un PID independiente para cada grado de libertad.
class PIDController:
    def __init__(self, num_joints, kp=1.0, ki=0.1, kd=0.05, limit=10.0):
        self.n = num_joints
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.limit = limit            # tope de salida (evita saturar)
        self.integral = [0.0] * num_joints
        self.prev_error = [0.0] * num_joints

    def reset(self):
        self.integral = [0.0] * self.n
        self.prev_error = [0.0] * self.n

    # update devuelve el vector de corrección para todas las juntas.
    def update(self, measurement, setpoint, dt):
        out = [0.0] * self.n
        for i in range(self.n):
            error = setpoint[i] - measurement[i]
            # Anti-windup: limitar la integral al rango de salida.
            self.integral[i] = max(-self.limit,
                                   min(self.limit, self.integral[i] + error * dt))
            deriv = (error - self.prev_error[i]) / dt
            self.prev_error[i] = error
            val = self.kp * error + self.ki * self.integral[i] + self.kd * deriv
            if val > self.limit:
                val = self.limit
            elif val < -self.limit:
                val = -self.limit
            out[i] = val
        return out
