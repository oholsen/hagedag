import math
from math import sin, cos, pi
from dataclasses import dataclass
from Plot import Plot

plot = Plot()


@dataclass
class State:
    x: float
    y: float
    theta: float


class Robot:

    def __init__(self):
        self.vR = 0
        self.vL = 0
        self.B = 0.3
        self.state = State(0, 0, 0)

    def speed(self):
        return (self.vR + self.vL) / 2

    def omega(self):
        return (self.vR - self.vL) / self.B
    
    def set_speed_omega(self, speed, omega):
        speed_delta = omega * self.B / 2
        self.vR = speed + speed_delta
        self.vL = 1 * (speed - speed_delta)

    def move(self, dt):
        v = self.speed()
        w = self.omega()
        dx_dt = v * cos(self.state.theta)
        dy_dt = v * sin(self.state.theta)
        self.state.x += dx_dt * dt
        self.state.y += dy_dt * dt
        self.state.theta += w * dt


def circle():
    robot = Robot()
    # circumference 2 pi R
    R = 20
    if 0:
        T = 10 # period
        speed = 2 * pi * R / T
        omega = 2 * pi / T
    else:
        speed = 0.5
        omega = speed / R
    dt = 0.1
    robot.state.y = -R
    robot.set_speed_omega(speed, omega)

    i = 0
    while True:
        robot.move(dt)
        if i % 20 == 0:
            print(i, robot.state.x, robot.state.y, robot.speed(), robot.state.theta)
            plot.update(robot)
        #time.sleep(self.dt)
        i += 1


def arc(robot, radius, speed, end_theta, direction):

    omega = speed / radius
    if not direction:
        omega = -omega
    robot.set_speed_omega(speed, omega)

    # stop at first zero crossing, but not a random jump +-pi
    dtheta_last = norm_angle(robot.state.theta - end_theta)

    i = 0
    dt = 0.1
    while True:
        dtheta = norm_angle(robot.state.theta - end_theta)
        if abs(dtheta) < 0.5:
            if dtheta_last < 0 and dtheta >= 0: break
            if dtheta_last > 0 and dtheta <= 0: break
        dtheta_last = dtheta
        robot.move(dt)
        if i % 10 == 0:
            plot.update(robot)
        #time.sleep(self.dt)
        i += 1

    print("ARC", direction, i, end_theta, robot.state.theta, dtheta, dtheta_last)


def norm_angle(a):
    while a > pi:
        a -= 2 * pi
    while a < -pi:
        a += 2 * pi
    return a


def fill():
    # TODO: need PID controller to catch offset from wheel slip

    robot = Robot()
    hline_diff = 1
    hline_y = -10.0

    dt = 0.1
    if 0:
        right = True
        robot.state.x = -20
    else:
        right = False
        robot.state.x = 20
    robot.state.y = 1
    robot.state.theta = 1

    i = 0
    while True:
        d = hline_y - robot.state.y
        # angle is 90 for large d
        _theta = (pi/2) * (1 - math.exp(-(2*d)**2))
        if d < 0:
            _theta = -_theta
        if not right:
            _theta = pi - _theta
        dtheta = norm_angle(_theta - robot.state.theta)
        # reduce speed if theta is very wrong, 1 at 0, 0.2 at pi/2
        speed = 0.5 * math.exp(-abs(5*dtheta)**2)
        # relax towards desired _theta
        omega = dtheta / 2
        robot.set_speed_omega(speed, omega)
        robot.move(dt)
        if i % 10 == 0:
            # print(d, _theta, dtheta, speed, omega, r.state.theta)
            # print(i, r.state.x, r.state.y, r.speed(), r.state.theta)
            plot.update(robot)
        # time.sleep(self.dt)
        i += 1

        # TODO: do half-circle, hit new line quickly
        if right and robot.state.x > 10:
            right = False
            hline_y += hline_diff
            arc(robot, hline_diff/2, 0.3, pi, True)
        elif not right and robot.state.x < -10:
            right = True
            hline_y += hline_diff
            arc(robot, hline_diff/2, 0.3, 0, False)
        if hline_y > 10:
            break

    plot.update(robot)
    plot.show()


def main():
    # circle()
    fill()


if __name__ == "__main__":
    main()
