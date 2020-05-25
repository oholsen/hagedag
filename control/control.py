import asyncio
import math
from math import sin, cos, pi
import cmath
from dataclasses import dataclass, replace
import numpy as np
import logging
import time
import random
from state import State
from PID import PID
from RobotModel import RobotModel
from Plotting import Plot
from abc import ABC


log = logging.getLogger(__name__)


class Control(ABC):

    def update(self, t: float, state): # -> (speed, omega)
        # state vectors [x y yaw v]'
        return None, None

    def end(self, t: float, state) -> bool:
        return False


class CompositeControl(Control):

    def __init__(self, controls):
        self.controls = iter(controls)
        self.control = next(self.controls)
        log.info("Starting %s", self.control)

    def update(self, t: float, state): # -> (speed, omega)
        # state vectors [x y yaw v]'
        assert self.controls is not None
        log.debug("Composite update %s %f %s", self.control, t, state)
        while self.control.end(t, state):
            try:
                self.control = next(self.controls)
            except StopAsyncIteration:
                self.control = None
                return None, None
            log.info("Starting %s %s", t, self.control)
        return self.control.update(t, state)

    def end(self, t: float, state) -> bool:
        return self.control is None


def norm_angle(a: float) -> float:
    while a > pi:
        a -= 2 * pi
    while a <= -pi:
        a += 2 * pi
    return a



class HLineControl(Control):

    def __init__(self, y: float, right: bool, end_x: float):
        self.pid = PID(0.4, 1, 1.5, 0.2)
        self.y = y
        self.right = right
        self.end_x = end_x
        self.t_last = None

    def __str__(self):
        return f"HLineControl({self.right},{self.end_x})"

    def reset(self):
        self.pid.clear()

    def update(self, t: float, state: State): # -> (speed, omega)
        # state vectors [x y yaw v]'
        d = self.y - state.y
        # angle is 90 for large d
        theta = (pi/2) * (1 - math.exp(-(2*d)**2))
        if d < 0:
            theta = -theta
        if not self.right:
            theta = pi - theta
        dtheta = norm_angle(theta - state.theta)
        # reduce speed if theta is very wrong, 1 at 0, 0.2 at pi/2
        speed = 0.2 * math.exp(-abs(5*dtheta)**2)
        # relax towards desired _theta
        omega = 0.2 * dtheta
        domega = 0

        
        if False and self.t_last is not None and abs(d) < 0.4:
            # only use PID if near the line
            # TODO: remove this discontinuity
            domega = -self.pid.update(d, t - self.t_last)
            if self.right:
                domega = -domega
        else:
            # TODO: reset PID? or update PID without using result?
            pass
        #print(t, d, theta, state.theta, dtheta, speed, domega)
        self.t_last = t
        return speed, omega + domega


    def end(self, t: float, state: State) -> bool:
        return self.right == (state.x >= self.end_x)




class PointControl(Control):

    def __init__(self, x: float, y: float, dist: float):
        self.x = x
        self.y = y
        self.dist = dist

    def __str__(self):
        return f"PointControl({self.x},{self.y})"

    def update(self, t: float, state: State): # -> (speed, omega)
        # state vectors [x y yaw v]'
        dx = self.x - state.x
        dy = self.y - state.y
        theta = math.atan2(dy, dx)
        dtheta = norm_angle(theta - state.theta)
        print("pos", state.x, state.y)
        print("theta", theta, dtheta)
        # reduce speed if theta is very wrong, 1 at 0, 0.2 at pi/2
        speed = 0.2 * math.exp(-abs(5*dtheta)**2)
        # relax towards desired _theta
        omega = 0.2 * dtheta
        print("cmd", speed, omega)
        return speed, omega

    def end(self, t: float, state: State) -> bool:
        dx = self.x - state.x
        dy = self.y - state.y
        d = math.hypot(dx, dy)
        print("distance", d, self.dist)
        return d < self.dist


class TimeControl(Control):

    def __init__(self, speed: float, omega: float, time: float):
        self.first = True
        self.speed = speed
        self.omega = omega
        self.time = time # seconds
        self.t0 = None

    def __str__(self):
        return f"TimeControl({self.time})"


    def update(self, t: float, state: State): # -> (speed, omega)
        if self.first:
            self.first = False
            self.t0 = t
            return self.speed, self.omega
        return None, None

    def end(self, t: float, state: State) -> bool:
        return self.t0 is not None and t >= self.t0 + self.time


def start_arc(radius, speed, direction):
    omega = speed / radius
    if not direction:
        omega = -omega
    return speed, omega

    
class ArcControl(Control):

    def __init__(self, speed, omega, end_theta):
        self.end_theta = end_theta
        self.dtheta_last = None
        self.first = True
        self.speed = speed
        self.omega = omega

    def __str__(self):
        return f"ArcControl({self.end_theta})"

    def update(self, t: float, state: State): # -> (speed, omega)
        if self.first:
            self.first = False
            return self.speed, self.omega
        return None, None

    def end(self, t: float, state: State) -> bool:
        # stop at first zero crossing, but not a random jump +-pi
        dtheta = norm_angle(state.theta - self.end_theta)
        if self.dtheta_last is None:
            self.dtheta_last = dtheta
            return False

        if abs(dtheta) < 0.5:
            if self.dtheta_last < 0 and dtheta >= 0: return True
            if self.dtheta_last > 0 and dtheta <= 0: return True
        self.dtheta_last = dtheta
        return False


"""
Program:

Based on coordinates in garden
hline_y = ...
hline_x0, x1 = ...

direction = based on initial position: right if x < (hline_x0 + hline_x1)/2 midpoint
hline -> random rot -> random line -> random rot 
swap direction
"""

def LineTest(x, y, xl, xr, y0):
    right = x < (xl + xr) / 2    
    end_x = {True: xr, False: xl}
    speed_max = 0.20
    omega_max = 0.20
    while True:
        yield HLineControl(y0, right, end_x[right])
        yield TimeControl(0, omega_max * random.uniform(-1, 1), 3)
        yield TimeControl(speed_max * random.uniform(0.1, 1), 0, 4)
        yield TimeControl(0, omega_max * random.uniform(-1, 1), 3)
        right = not right


async def simulate_line():
    model = RobotModel(State(1, 1, 0, 0))
    control = CompositeControl(LineTest(1, 1, -5, 5, 0))
    plot = Plot()
    dt = 1
    t = 0.0
    state = model.get_state()
    plot.update(state)
    while not control.end(t, state):

        t += dt
        model.update(dt)
        state = model.get_state()
        plot.update(state)
        
        speed, omega = control.update(t, state)
        if speed is not None:
            model.set_speed_omega(speed, omega)


async def simulate_point():
    model = RobotModel(State(1, 1, 0, 0))
    control = PointControl(-5, 2, 0.2)
    plot = Plot()
    dt = 0.1
    t = 0.0
    state = model.get_state()
    plot.update(state)
    while not control.end(t, state):

        t += dt
        model.update(dt)
        state = model.get_state()
        plot.update(state)
        
        speed, omega = control.update(t, state)
        if speed is not None:
            model.set_speed_omega(speed, omega)


"""

M: t1, u -> t2, u, z
E: t2, u, z -> t2, s_est
C: t2, s_est -> t2, u (t2 goes to M as t1)

Observation delay in u

stream u, z from "record"
stream s_est from "tracking"
stream u from "control"

"""


def main():
    # asyncio.run(simulate_line())
    asyncio.run(simulate_point())


if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        #filename="control.log",
        format='%(asctime)s %(levelname)s %(message)s',
    )
    main()
