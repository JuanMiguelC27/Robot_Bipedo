# Estima la orientación a partir de la lectura cruda de la IMU.
class IMUProcessor:
    def __init__(self):
        self.roll = 0.0
        self.pitch = 0.0
        self.accel_z = 0.0

    def integrate(self, imu_raw):
        self.roll += imu_raw.get('gyro', {}).get('x', 0.0) * 0.02
        self.pitch += imu_raw.get('gyro', {}).get('y', 0.0) * 0.02
        self.accel_z = imu_raw.get('accel', {}).get('z', 0.0)
        return {'roll': self.roll, 'pitch': self.pitch, 'accel_z': self.accel_z}
