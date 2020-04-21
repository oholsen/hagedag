import math
from math import sin, cos, pi
from Plotting import Plot
from PID import PID
from state import State
from dataclasses import replace

plot = Plot()

# e.g. "slipping" if diameter of left wheel is smaller
slip_left = 0.0
# slip_left = 0.05

"""
TODO:
- Estimate heading - from odometry - with noise
- Test error in position estimate
- Fuse GPS and odometry, with noise in GPS and odometry - and systematic error in odometry

Heading estimate:
- based on GPS path, extrapolate with odometry
- e.g. complementary filter with high-pass on odometry and low-pass from GPS


omega_odometry = delta wheel speed / wheel_base
heading_gps = delta GPS pos over the last T seconds
If the  delta GPS pos is small, then update based on odometry only, e.g. if the robot turns around it's own axis
alpha is based on dt - 

heading = (1-alpha)*(heading + omega_odometry * dt) + alpha * heading_gps

"""

class Estimator:

    def update(self, dt: float, state: State):
        pass

    def state(self) -> State:
        pass


class ExactEstimator(Estimator):

    def __init__(self, state: State):
        self._state = state

    def update(self, dt: float, state: State):
        self._state = state

    def state(self) -> State:
        return self._state


class HeadingEstimator(Estimator):

    def __init__(self, state: State):
        self._state = state

    def update(self, dt: float, state: State):
        self._state = state

    def state(self) -> State:
        state = replace(self._state)
        # TODO: calculate theta
        return state
        #return self._state


class Robot:
    # physical state

    def __init__(self, state: State, estimator: Estimator):
        self.B = 0.3
        self.t = 0.0
        self.t_plot: float = None
        self.state = state 
        self.estimator = estimator
        self.vR = 0.0
        self.vL = 0.0
        self.set_speed_omega(state.speed, state.omega)

    def set_speed_omega(self, speed: float, omega: float):
        speed_delta = omega * self.B / 2
        vR = speed + speed_delta
        vL = (1 - slip_left) * (speed - speed_delta)
        self.state.speed = (vR + vL) / 2.0
        self.state.omega = (vR - vL) / self.B

    def update(self, dt):
        v = self.state.speed
        w = self.state.omega
        dx_dt = v * cos(self.state.theta)
        dy_dt = v * sin(self.state.theta)
        self.state.x += dx_dt * dt
        self.state.y += dy_dt * dt
        self.state.theta += w * dt
        self.state.speed = v
        self.state.omega = w
        self.t += dt
        self.estimator.update(dt, self.state)
        if self.t_plot is None or self.t > self.t_plot + 1:
            plot.update(self.state) 
            self.t_plot = self.t
        # time.sleep(self.dt)


class Control:

    def __init__(self, robot: Robot):
        self.robot = robot
        self.pid = PID(0.4, 2, 1.5, 0.2)

    def state(self) -> State:
        return self.robot.estimator.state()

    def reset(self):
        self.pid.clear()

    def update_hline(self, dt, y, right):
        state = self.state()
        d = y - state.y
        # angle is 90 for large d
        _theta = (pi/2) * (1 - math.exp(-(2*d)**2))
        if d < 0:
            _theta = -_theta
        if not right:
            _theta = pi - _theta
        dtheta = norm_angle(_theta - state.theta)
        # reduce speed if theta is very wrong, 1 at 0, 0.2 at pi/2
        speed = 0.5 * math.exp(-abs(5*dtheta)**2)
        # relax towards desired _theta
        omega = dtheta / 2
        domega = 0
        if abs(d) < 0.4:
            domega = -self.pid.update(d, dt)
            if right:
                domega = -domega
        else:
            # TODO: reset PID? or update PID without using result?
            pass
        self.robot.set_speed_omega(speed, omega + domega)
        self.robot.update(dt)


    def arc(self, dt, radius, speed, end_theta, direction):

        omega = speed / radius
        if not direction:
            omega = -omega
        self.robot.set_speed_omega(speed, omega)

        # stop at first zero crossing, but not a random jump +-pi
        dtheta_last = norm_angle(self.state().theta - end_theta)

        while True:
            dtheta = norm_angle(self.state().theta - end_theta)
            if abs(dtheta) < 0.5:
                if dtheta_last < 0 and dtheta >= 0: break
                if dtheta_last > 0 and dtheta <= 0: break
            dtheta_last = dtheta
            self.robot.update(dt)

        # print("ARC", direction, i, end_theta, robot.state.theta, dtheta, dtheta_last)


def circle():
    R = 20
    speed = 0.5
    omega = speed / R # anti-clockwise
    state = State(0, -R, 0, speed, omega) # exact state
    robot = Robot(state, None)
    dt = 0.1
    while True:
        robot.update(dt)


def norm_angle(a):
    while a > pi:
        a -= 2 * pi
    while a < -pi:
        a += 2 * pi
    return a


def fill():
    arc_speed = 0.3
    hline_diff = 1
    hline_y = -10.0
    hline_x = 15
    dt = 0.1

    state = State(0, 0, 0, 0, 0) # exact state

    if 0:
        right = True
        state.x = -20
    else:
        right = False
        state.x = 0
    state.y = -5
    state.theta = 1

    right = True
    state.theta = 0.001
    state.y = hline_y + 0.01
    state.x = -hline_x + 0.5
    
    robot = Robot(state, ExactEstimator(state))
    control = Control(robot)

    i = 0
    while True:
        control.update_hline(dt, hline_y, right)
        if i % 10 == 0:
            # print(d, _theta, dtheta, speed, omega, r.state.theta)
            # print(i, r.state.x, r.state.y, r.speed(), r.state.theta)
            # print(dtheta, d, domega)
            # print(d, dtheta, speed, omega, domega)
            # print(x, y, theta, speed, omega, domega)
            # plot.update(robot)
            pass
        # time.sleep(self.dt)
        i += 1

        state = robot.estimator.state()
        if right and state.x >= hline_x:
            right = False
            hline_y += hline_diff
            control.arc(dt, hline_diff/2, arc_speed, pi, True)
            control.reset()
        elif not right and state.x <= -hline_x:
            right = True
            hline_y += hline_diff
            control.arc(dt, hline_diff/2, arc_speed, 0, False)
            control.reset()
        if hline_y > 10:
            break

    plot.update(robot)
    plot.show()


def main():
    # circle()
    fill()


if __name__ == "__main__":
    main()
