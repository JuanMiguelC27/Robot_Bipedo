# Equilibrio del cuerpo usando roll/pitch de la IMU.
# Se usa cuando el robot ya tiene las dos piernas e IMU real.
class BalanceController:
    def __init__(self, num_joints):
        self.n = num_joints

    def compute_correction(self, imu):
        correction = [0.0] * self.n
        roll = imu.get('roll', 0.0)
        pitch = imu.get('pitch', 0.0)
        if abs(roll) > 0.05:
            correction[0] -= roll * 0.5
        if abs(pitch) > 0.05:
            correction[1] -= pitch * 0.5
        return correction
