class PID:
    """PID controller."""

    def __init__(self, Kp, Ti, Td, Imax):
        self.Kp = Kp
        self.Ti = Ti
        self.Td = Td
        self.Imax = Imax
        self.clear()

    def clear(self):
        self.Cp = 0.0
        self.Ci = 0.0
        self.Cd = 0.0
        self.previous_error = 0.0

    def update(self, error, dt):
        de = error - self.previous_error
        self.previous_error = error

        self.Cp = self.Kp * error
        self.Ci += self.Kp * error * dt / self.Ti
        self.Ci = min(self.Ci, self.Imax)
        self.Ci = max(self.Ci, -self.Imax)
        self.Cd = self.Kp * self.Td * de / dt

        return self.Cp + self.Ci + self.Cd
