from math import cos, sin
from state import State
import RobotMessages

# slip_left = 0.0

class RobotModel:
    # physical state

    def __init__(self, state: State):
        self.B = RobotMessages.WHEEL_BASE
        self.state = state
        self.omega = 0

    def set_speed_omega(self, speed: float, omega: float):
        # speed_delta = omega * self.B / 2
        # vR = speed + speed_delta
        # vL = (1 - slip_left) * (speed - speed_delta)
        self.state.speed = speed
        self.omega = omega

    def update(self, dt: float):
        v = self.state.speed
        dx_dt = v * cos(self.state.theta)
        dy_dt = v * sin(self.state.theta)
        self.state.x += dx_dt * dt
        self.state.y += dy_dt * dt
        self.state.theta += self.omega * dt
        return self.state

    def get_state(self) -> State:
        return self.state

    def command(self, cmd: RobotMessages.RobotCommand):
        if isinstance(cmd, RobotMessages.SpeedCommand):
            self.state.speed = cmd.speed
        elif isinstance(cmd, RobotMessages.OmegaCommand):
            self.state.omega = cmd.omega
        elif isinstance(cmd, RobotMessages.StopCommand):
            self.state.speed = 0
            self.state.omega = 0
