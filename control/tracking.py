"""

Based on Extended kalman filter (EKF) localization sample in PythonRobotics by Atsushi Sakai (@Atsushi_twi)

"""

import math
import matplotlib.pyplot as plt
import numpy as np


#  Simulation parameter
INPUT_NOISE = np.diag([0.1, np.deg2rad(30.0)]) ** 2
GPS_NOISE = np.diag([0.03, 0.03]) ** 2


# Covariance for EKF simulation
Q = np.diag([
    0.02,  # variance of location on x-axis
    0.02,  # variance of location on y-axis
    np.deg2rad(10.0),  # variance of yaw angle
    0.1  # variance of velocity
]) ** 2  # predict state covariance

# Observation x,y position covariance
R = np.diag([0.02, 0.02]) ** 2  


if 0:    
    # Covariance for EKF simulation
    Q = np.diag([
        0.1,  # variance of location on x-axis
        0.1,  # variance of location on y-axis
        np.deg2rad(1.0),  # variance of yaw angle
        1.0  # variance of velocity
    ]) ** 2  # predict state covariance
    R = np.diag([1.0, 1.0]) ** 2  # Observation x,y position covariance


def calc_input():
    v = 1.0  # [m/s]
    yawrate = 0.1  # [rad/s]
    u = np.array([[v], [yawrate]])
    return u


def simulate(xTrue, u, dt: float):
    xTrue = motion_model(xTrue, u, dt)

    # add noise to gps x-y
    z = observation_model(xTrue) + GPS_NOISE @ np.random.randn(2, 1)

    # add noise to input
    ud = u + INPUT_NOISE @ np.random.randn(2, 1)

    return xTrue, z, ud


def dead_reckon(xd, ud, dt: float):
    x = motion_model(xd, ud, dt)
    # print("DR", xd, ud, dt, x)
    return x


def observation(xTrue, xd, u, dt: float):
    xTrue = motion_model(xTrue, u, dt)

    # add noise to gps x-y
    z = observation_model(xTrue) + GPS_NOISE @ np.random.randn(2, 1)

    # add noise to input
    ud = u + INPUT_NOISE @ np.random.randn(2, 1)

    xd = motion_model(xd, ud, dt)

    return xTrue, z, xd, ud


def motion_model(x, u, dt: float):
    F = np.array([[1.0, 0, 0, 0],
                  [0, 1.0, 0, 0],
                  [0, 0, 1.0, 0],
                  [0, 0, 0, 0]])

    B = np.array([[dt * math.cos(x[2, 0]), 0],
                  [dt * math.sin(x[2, 0]), 0],
                  [0.0, dt],
                  [1.0, 0.0]])

    x = F @ x + B @ u
    return x


def observation_model(x):
    H = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0]
    ])

    z = H @ x
    return z


def jacob_f(x, u, DT: float):
    """
    Jacobian of Motion Model

    motion model
    x_{t+1} = x_t+v*dt*cos(yaw)
    y_{t+1} = y_t+v*dt*sin(yaw)
    yaw_{t+1} = yaw_t+omega*dt
    v_{t+1} = v{t}
    so
    dx/dyaw = -v*dt*sin(yaw)
    dx/dv = dt*cos(yaw)
    dy/dyaw = v*dt*cos(yaw)
    dy/dv = dt*sin(yaw)
    """
    yaw = x[2, 0]
    v = u[0, 0]
    jF = np.array([
        [1.0, 0.0, -DT * v * math.sin(yaw), DT * math.cos(yaw)],
        [0.0, 1.0, DT * v * math.cos(yaw), DT * math.sin(yaw)],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0]])

    return jF


def jacob_h():
    # Jacobian of Observation Model
    jH = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0]
    ])

    return jH


def ekf_estimation(xEst, PEst, z, u, dt: float):
    #  Predict
    xPred = motion_model(xEst, u, dt)
    jF = jacob_f(xEst, u, dt)
    PPred = jF @ PEst @ jF.T + Q

    #  Update
    jH = jacob_h()
    zPred = observation_model(xPred)
    y = z - zPred
    S = jH @ PPred @ jH.T + R
    K = PPred @ jH.T @ np.linalg.inv(S)
    xEst = xPred + K @ y
    PEst = (np.eye(len(xEst)) - K @ jH) @ PPred
    return xEst, PEst


def plot_covariance_ellipse(xEst, PEst):  # pragma: no cover
    Pxy = PEst[0:2, 0:2]
    eigval, eigvec = np.linalg.eig(Pxy)

    if eigval[0] >= eigval[1]:
        bigind = 0
        smallind = 1
    else:
        bigind = 1
        smallind = 0

    t = np.arange(0, 2 * math.pi + 0.1, 0.1)
    a = math.sqrt(eigval[bigind])
    b = math.sqrt(eigval[smallind])
    x = [a * math.cos(it) for it in t]
    y = [b * math.sin(it) for it in t]
    angle = math.atan2(eigvec[bigind, 1], eigvec[bigind, 0])
    rot = np.array([[math.cos(angle), math.sin(angle)],
                    [-math.sin(angle), math.cos(angle)]])
    fx = rot @ (np.array([x, y]))
    px = np.array(fx[0, :] + xEst[0, 0]).flatten()
    py = np.array(fx[1, :] + xEst[1, 0]).flatten()
    plt.plot(px, py, "--r")


async def read_simulation():
    dt = 0.1  # time tick [s]
    SIM_TIME = 50.0  # simulation time [s]
    time = 0.0

    np.random.seed(23)
    # State Vector [x y yaw v]'
    xTrue = np.zeros((4, 1))
    z = np.zeros((2, 1))
    yield 0, None, z, None

    while SIM_TIME >= time:
        u = calc_input()
        xTrue, z, ud = simulate(xTrue, u, dt)
        time += dt
        yield time, dt, z, ud


class History:
    def __init__(self, state=None):
        self.history = state

    def add(self, x):
        if self.history is None:
            self.history = x
        else:
            self.history = np.hstack((x, self.history))

    def plot(self, fmt):
        plt.plot(self.history[0, :], self.history[1, :], fmt)

    def plot_flatten(self, fmt):
        plt.plot(self.history[0, :].flatten(),
                self.history[1, :].flatten(), fmt)


class DeadReckonTracker:
    def __init__(self, state=None):
        # state vectors [x y yaw v]'
        self.state = state

    def init(self, state):
        self.state = state

    def get_state(self):
        return self.state

    def update(self, z, u, dt: float):
        # u: input, z: observation (not used here)
        self.state = dead_reckon(self.state, u, dt)
        return self.state


class ExtendedKalmanFilterTracker:
    def __init__(self, state=None):
        # state vectors [x y yaw v]'
        self.state = state
        self.P = np.eye(4)

    def init(self, state):
        self.state = state
        self.P = np.eye(4)

    def get_state(self):
        return self.state

    def update(self, z, u, dt: float):
        # u: input, z: observation (not used here)
        self.state, self.P = ekf_estimation(self.state, self.P, z, u, dt)
        return self.state

    def plot_covariance(self):
        plot_covariance_ellipse(self.state, self.P)


async def track(stream, yaw=0, speed=0):
    show_animation = True
    # show_animation = False
    
    # state vectors [x y yaw v]'
    first = True

    def plot():
        # plt.gca().invert_xaxis()
        # plt.gca().invert_yaxis()
        plt.axis("equal")
        plt.grid(True)
        hz.plot(".-g")
        # hdr.plot_flatten("-k")            
        hekf.plot_flatten("-r")
        ekf.plot_covariance()

        # State Vector [x y yaw v]'
        s = ekf.get_state().flatten()
        # print("STATE", s)
        x = s[0]
        y = s[1]
        yaw = s[2]
        # speed = s[3]
        a = 1  # * speed
        plt.arrow(x, y, a * math.cos(yaw), a * math.sin(yaw))


    # async for o in stream: print("track", repr(o))

    async for _, dt, z, ud in stream:

        # print("TRACK STREAM", dt, z, ud)
        if first:
            # init state with the first observation, using yaw, v = 0
            s = np.array([[z[0][0]], [z[1][0]], [yaw], [speed]])
            dr = DeadReckonTracker(s)
            hdr = History(s)
            ekf = ExtendedKalmanFilterTracker(s)
            hekf = History(s)
            hz = History(z)
            first = False
            yield s
            continue

        hdr.add(dr.update(z, ud, dt))
        s = ekf.update(z, ud, dt)
        hekf.add(s)
        hz.add(z)
        yield s

        if show_animation:
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect('key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
            plot()
            plt.pause(0.001)

    plot()
    plt.show()


if __name__ == '__main__':
    import asyncio
    asyncio.run(track(read_simulation()))
