class OrientationData:
    def __init__(self, qx, qy, qz, qw, x, y, z, var_qx, var_qy, var_qz, var_qw):
        self.qx, self.qy, self.qz, self.qw = qx, qy, qz, qw
        self.x, self.y, self.z = x, y, z
        self.var_qx, self.var_qy, self.var_qz, self.var_qw = var_qx, var_qy, var_qz, var_qw