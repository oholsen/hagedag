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

# Observation x,y position covariance, now dynamic from receiver (input stream)
# R = np.diag([0.02, 0.02]) ** 2  


def calc_input():
    v = 1.0  # [m/s]
    yawrate = 0.1  # [rad/s]
    return np.array([[v], [yawrate]])


def simulate(xTrue, u, dt: float):
    xTrue = motion_model(xTrue, u, dt)

    # add noise to gps x-y
    z = observation_model(xTrue) + GPS_NOISE @ np.random.randn(2, 1)

    # add noise to input
    ud = u + INPUT_NOISE @ np.random.randn(2, 1)

    return xTrue, z, ud


def observation(x_true, xd, u, dt: float):
    # simulation
    x_true = motion_model(x_true, u, dt)

    # add noise to gps x-y
    z = observation_model(x_true) + GPS_NOISE @ np.random.randn(2, 1)

    # add noise to input
    ud = u + INPUT_NOISE @ np.random.randn(2, 1)

    xd = motion_model(xd, ud, dt)

    return x_true, z, xd, ud


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


def ekf_estimation(x_est, P_est, z, u, R, dt: float):
    #  Predict
    x_pred = motion_model(x_est, u, dt)
    jF = jacob_f(x_est, u, dt)
    P_pred = jF @ P_est @ jF.T + Q

    #  Update
    jH = jacob_h()
    z_pred = observation_model(x_pred)
    y = z - z_pred
    S = jH @ P_pred @ jH.T + R
    K = P_pred @ jH.T @ np.linalg.inv(S)
    x_est = x_pred + K @ y
    P_est = (np.eye(len(x_est)) - K @ jH) @ P_pred
    return x_est, P_est


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
    hdop = 0.1
    np.random.seed(23)
    # State Vector [x y yaw v]'
    x_true = np.zeros((4, 1))
    z = np.zeros((2, 1))
    yield 0, None, z, None, None

    while time <= SIM_TIME:
        u = calc_input()
        x_true, z, ud = simulate(x_true, u, dt)
        time += dt
        yield time, dt, z, ud, hdop


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
        self.state = motion_model(self.state, u, dt)
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

    def update(self, z, u, R, dt: float):
        # u: input, z: observation (not used here)
        self.state, self.P = ekf_estimation(self.state, self.P, z, u, R, dt)
        return self.state

    def plot_covariance(self):
        plot_covariance_ellipse(self.state, self.P)


async def track(stream, yaw=0, speed=0):
    #show_animation = True
    show_animation = False
    
    # state vectors [x y yaw v]'
    first = True

    def plot():
        # plt.gca().invert_xaxis()
        # plt.gca().invert_yaxis()
        plt.axis("equal")
        plt.grid(True)
        hz.plot(".g")
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

    async for _, dt, z, ud, hdop in stream:

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

        # each component is 0.707 * hdop (hdop is radius)
        R = np.diag([0.7 * hdop, 0.7 * hdop]) ** 2  
        hdr.add(dr.update(z, ud, dt))
        s = ekf.update(z, ud, R, dt)
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


async def main():
    async for s in track(read_simulation()):
        print(s)


if __name__ == '__main__':
    import asyncio
    asyncio.run(main())
