import math
from math import sin, cos, pi
import cmath
from Plotting import Plot
from PID import PID
from state import State
from dataclasses import replace
import numpy as np
from statistics import leastsq

plot = Plot()

# e.g. "slipping" if diameter of left wheel is smaller
slip_left = 0.0
# slip_left = 0.05
# FIXME: does not handle any error here, e.g. 1.01
omega_scale = 1.01

"""
TODO:
- Estimate heading - from odometry - with noise
- Test error in position estimate
- Fuse GPS and odometry, with noise in GPS and odometry - and systematic error in odometry

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
    # Complementary filter with high-pass on odometry and low-pass from GPS.

    alpha = 0.3
    min_distance = 0.2

    def __init__(self, state: State):
        self._state: State = replace(state)
        self.t = 0.0
        self.positions = []

    def heading(self, dt: float, state: State):
        omega_odometry = omega_scale * state.omega
        theta = self._state.theta
        theta_odo = norm_angle(theta + omega_odometry * dt)
        if abs(omega_odometry) < 0.1 and len(self.positions) > 1:
            t0, p0 = self.positions[0]
            t1, p1 = self.positions[-1]
            dt: float = t1 - t0
            d: complex = p1 - p0
            # check if good linear fit, avoid lagging heading in turns
            xs = np.array([p[1].real for p in self.positions])
            ys = np.array([p[1].imag for p in self.positions])
            fit = leastsq(xs, ys)            
            print("FIT", len(self.positions), fit, dt, abs(d))
            if fit and abs(abs(fit[2]) - 1) < 0.05 and abs(dt) > 0.4 and abs(d) >= self.min_distance:
                theta_gps = norm_angle(cmath.phase(d))
                print("HDG FILTER", dt, fit, d, len(xs), theta, omega_odometry, theta_gps, theta_odo)
                theta = norm_angle((1 - self.alpha) * theta_odo + self.alpha * theta_gps)
                return theta

        # TODO: use position encoders difference...
        print("HDG ODO", theta_odo);
        return theta_odo


    def update(self, dt: float, state: State):
        self.t += dt
        # TODO: estimate position
        self.positions.append((self.t, complex(state.x, state.y)))
        while self.positions[0][0] < self.t - 0.5:
            self.positions.pop(0)
        theta = self.heading(dt, state)
        if 0:
            # No fusion, just error:
            omega_odometry = omega_scale * state.omega
            theta = self._state.theta + omega_odometry * dt

        # print(self.t, state.omega, theta, state.theta, norm_angle(theta - state.theta))
        self._state = replace(state, theta=theta)
        error = norm_angle(self._state.theta - state.theta)
        print("HDG", self._state.theta, state.theta, theta, error)

        #if abs(error) > 0.2: raise SystemExit
        # print(self.t, d, abs(d), cmath.phase(d), state.theta)
    
    def state(self) -> State:
        return self._state 


class Robot:
    # physical state

    def __init__(self, state: State, estimator: Estimator):
        self.B = 0.3
        self.t = 0.0
        self.t_plot: float = None
        self.state = state 
        self.estimator = estimator
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
        self.pid = PID(0.4, 1, 1.5, 0.2)

    def state(self) -> State:
        return self.robot.estimator.state()

    def reset(self):
        self.pid.clear()

    def update_hline(self, dt, y, right):
        state = self.state()
        d = y - state.y
        # angle is 90 for large d
        theta = (pi/2) * (1 - math.exp(-(2*d)**2))
        if d < 0:
            theta = -theta
        if not right:
            theta = pi - theta
        dtheta = norm_angle(theta - state.theta)
        # reduce speed if theta is very wrong, 1 at 0, 0.2 at pi/2
        speed = 0.5 * math.exp(-abs(5*dtheta)**2)
        # relax towards desired _theta
        omega = dtheta # / 2
        domega = 0
        if abs(d) < 0.4:
            domega = -self.pid.update(d, dt)
            if right:
                domega = -domega
        else:
            # TODO: reset PID? or update PID without using result?
            pass
        print(d, theta, state.theta, dtheta, speed, domega)
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
    while a <= -pi:
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

    if 1:
        right = True
        state.theta = 0.001
    else:
        right = False
        state.theta = pi + 0.001
    state.y = hline_y + 0.01
    state.x = -hline_x + 0.5
    state.x = hline_x - 5
    
    robot = Robot(state, HeadingEstimator(state))
    control = Control(robot)

    print("hline")
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
            print("arc")
            control.arc(dt, hline_diff/2, arc_speed, pi, True)
            control.reset()
            print("hline")
        elif not right and state.x <= -hline_x:
            right = True
            hline_y += hline_diff
            print("arc")
            control.arc(dt, hline_diff/2, arc_speed, 0, False)
            control.reset()
            print("hline")
        if hline_y > 10:
            break

    plot.update(robot)
    plot.show()


def main():
    # circle()
    fill()


if __name__ == "__main__":
    main()
