class MotorController:
    def __init__(self, num_joints):
        self.n = num_joints

    # apply suma consigna + corrección y entrega el comando por junta.
    def apply(self, targets, corrections):
        return [targets[i] + corrections[i] for i in range(self.n)]