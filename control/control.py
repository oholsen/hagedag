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

class ControlTime(ABC):
    def tick(self) -> None:
        pass
    def time(self) -> float:
        pass
    def dt(self) -> float:
        pass


class RealTime(ControlTime):
    def __init__(self):
        self.t = time.time()
        self._last_dt = None

    def tick(self) -> None:
        t = time.time()
        self._last_dt = t - self.t
        self.t = t

    def time(self):
        return self.t

    def dt(self) -> float:
        return self._last_dt


class SimulatedTime(ControlTime):
    def __init__(self, dt: float):        
        self.t = 0
        self._dt = dt

    def tick(self) -> None:
        self.t += self._dt

    def time(self):
        return self.t

    def dt(self) -> float:
        return self._dt


control_time = SimulatedTime(0.1)


class Control(ABC):

    def update(self, state): # -> (speed, omega)
        # state vectors [x y yaw v]'
        return None, None

    def end(self, state) -> bool:
        return False


# Feedback can be incremental, eg gps and robot state out of synch - don't support that
# TODO: async sequence of states instead of get_state
# but want to reuse same state for end to init the next control.
async def control_loop(controls, get_state, update):
    def report():
        #print(control_time.time(), "STATE", state.x, state.y, state.theta, state.speed)
        pass
    state = await get_state()
    report()
    for control in controls:
        log.info("%s Starting %s", control_time.time(), control)
        while not control.end(state):
            control_time.tick()
            speed, omega = control.update(state)
            # print(control_time.time(), "UPDATE", speed, omega)
            update(speed, omega)
            state = await get_state()
            report()


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

    def __str__(self):
        return f"HLineControl({self.right},{self.end_x})"

    def reset(self):
        self.pid.clear()

    def update(self, state: State): # -> (speed, omega)
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
        speed = 0.5 * math.exp(-abs(5*dtheta)**2)
        # relax towards desired _theta
        omega = dtheta # / 2
        domega = 0
        if False and abs(d) < 0.4:
            # only use PID if near the line
            # TODO: remove this discontinuity
            domega = -self.pid.update(d, control_time.dt())
            if self.right:
                domega = -domega
        else:
            # TODO: reset PID? or update PID without using result?
            pass
        #print(control_time.time(), d, theta, state.theta, dtheta, speed, domega)
        return speed, omega + domega


    def end(self, state: State) -> bool:
        return self.right == (state.x >= self.end_x)


class TimeControl(Control):

    def __init__(self, speed: float, omega: float, time: float):
        self.first = True
        self.speed = speed
        self.omega = omega
        self.time = time # seconds
        self.t0 = None

    def __str__(self):
        return f"TimeControl({self.time})"


    def update(self, state: State): # -> (speed, omega)
        if self.first:
            self.first = False
            self.t0 = control_time.time()
            return self.speed, self.omega
        return None, None

    def end(self, state: State) -> bool:
        return self.t0 is not None and control_time.t >= self.t0 + self.time


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

    def update(self, state: State): # -> (speed, omega)
        if self.first:
            self.first = False
            return self.speed, self.omega
        return None, None

    def end(self, state: State) -> bool:
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
    speed_max = 0.5
    omega_max = 0.5
    while True:
        yield HLineControl(y0, right, end_x[right])
        yield TimeControl(0, omega_max * random.uniform(-1, 1), 3)
        yield TimeControl(speed_max * random.uniform(0.1, 1), 0, 4)
        yield TimeControl(0, omega_max * random.uniform(-1, 1), 3)
        right = not right


async def simulate():
    model = RobotModel(State(1, 1, 0, 0))
    controls = LineTest(1, 1, -5, 5, 0)
    plot = Plot()

    async def get_state():
        return model.get_state()

    def update(speed, omega):
        if speed is not None:
            model.set_speed_omega(speed, omega)
        model.update(control_time.dt())
        plot.update(model.get_state())

    await control_loop(controls, get_state, update)


async def real():
    model = RobotModel(State(1, 1, 0, 0))
    controls = LineTest(1, 1, -5, 5, 0)
    plot = Plot()

    async def get_state():
        return model.get_state()

    def update(speed, omega):
        if speed is not None:
            model.set_speed_omega(speed, omega)
        model.update(control_time.dt())
        plot.update(model.get_state())

    await asyncio.gather(control_loop(controls, get_state, update))



def main():
    asyncio.run(simulate())





if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        #filename="control.log",
        format='%(asctime)s %(levelname)s %(message)s',
    )
    main()
