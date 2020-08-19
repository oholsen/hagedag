from math import cos, sin
from state import State

# slip_left = 0.0

WHEEL_BASE         = 0.3300 # m
MAX_TICK_SPEED     = 5500 # max tick speed is about 5800
DIST_PER_TICK      = 1.145 / 20550 # m
MAX_SPEED          = MAX_TICK_SPEED * DIST_PER_TICK # m/s
# MAX_SPEED          = 0.15 # m/s, probably slightly higher, but to guarantee heading...

"""
Calibration on office floor:
20550 ticks
114.5 cm
"""


class RobotModel:
    # physical state

    def __init__(self, state: State):
        # self.B = WHEEL_BASE
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
