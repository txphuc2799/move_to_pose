class PIDController(object):
    
    def __init__(self, dt=0.1, kP=0.0, kI=0.0, kD=0.0):
        self.dt = dt
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.prev_error = 0.0
        self.integral = 0.0

    def compute(self, error):
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        output = self.kP * error + self.kI * self.integral + self.kD * derivative
        self.prev_error = error

        return output