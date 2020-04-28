"""
Particle Filter  based on PythonRobotics by Atsushi Sakai (@Atsushi_twi).
Need to estimate heading (yaw) based on odometry and precise localization in order to control the robot.

TODO: figure out if putting the GPS off the wheel axis helps. Can resolve the robot spinning.


x is state  [x y yaw v]'
u is action [v yaw_rate]'

observation is [x y]'

"""

import math

import matplotlib.pyplot as plt
import numpy as np


# Q,R are for action u
# Estimation parameter of PF
Q_est = np.diag([0.02]) ** 2  # GPS precision error
R_est = np.diag([0.8, np.deg2rad(20.0)]) ** 2  # input error

#  Simulation parameters - noise added to observations
Q_sim = np.diag([0.02]) ** 2
R_sim = np.diag([0.4, np.deg2rad(10.0)]) ** 2

# Particle filter parameter
NP = 1000  # Number of Particle
NTh = NP / 5.0  # Number of particle for re-sampling


def calc_input():
    v = 1.0  # [m/s]
    yaw_rate = 0.1  # [rad/s]
    u = np.array([[v, yaw_rate]]).T
    return u


def observation(x_true, xd, u, dt):
    # simulate a step: 
    # - x_true simulated true state
    # - xd dead reckoning - for comparison only
    # - u action input
    
    # exact one step forward 
    x_true = motion_model(x_true, u, dt)

    # add noise to gps x-y
    x = x_true[0, 0] + np.random.randn() * Q_sim[0, 0] ** 0.5  # add noise
    y = x_true[1, 0] + np.random.randn() * Q_sim[0, 0] ** 0.5  # add noise
    z = (x, y)

    # add noise to input
    ud1 = u[0, 0] + np.random.randn() * R_sim[0, 0] ** 0.5
    ud2 = u[1, 0] + np.random.randn() * R_sim[1, 1] ** 0.5
    ud = np.array([[ud1, ud2]]).T

    # dead reckoning: estimage one step forward (from estimated state)
    xd = motion_model(xd, ud, dt)

    return x_true, z, xd, ud


def motion_model(x, u, dt: float):
    # state += except v
    F = np.array([[1.0, 0, 0, 0],
                  [0, 1.0, 0, 0],
                  [0, 0, 1.0, 0],
                  [0, 0, 0, 0]])


    # x is state  [x y yaw v]'
    # u is action [v yaw_rate]'

    # x += cos(yaw) * v * dt
    # y += sin(yaw) * v * dt
    # yaw += yaw_rate * dt
    # v = v

    B = np.array([[dt * math.cos(x[2, 0]), 0],
                  [dt * math.sin(x[2, 0]), 0],
                  [0.0, dt],
                  [1.0, 0.0]])

    x = F.dot(x) + B.dot(u)

    return x


def gauss_likelihood(x, sigma):
    p = 1.0 / math.sqrt(2.0 * math.pi * sigma ** 2) * \
        math.exp(-x ** 2 / (2 * sigma ** 2))

    return p


def calc_covariance(x_est, px, pw):
    """
    calculate covariance matrix
    see ipynb doc
    """
    cov = np.zeros((3, 3))
    n_particle = px.shape[1]
    for i in range(n_particle):
        dx = (px[:, i:i + 1] - x_est)[0:3]
        cov += pw[0, i] * dx @ dx.T
    cov *= 1.0 / (1.0 - pw @ pw.T)

    return cov


def pf_localization(px, pw, z, u, dt: float):
    """
    Localization with Particle filter

    px is particle states
    pw is particle weights
    z is beacon positions and observed distance (with error)
    u is action
    """

    # for each particle:
    #   x is state
    #   w is weight: multiplies through each iteration for each particle
    #   ud is action with noise
    #   x is estimated new state
    #   for each beacon:
    #      USING EXACT DISTANCE TO CALC DZ????
    #      w *= gauss(dz, sqrt(Q0))
    #   px[i] = x
    #   pw[i] = w

    for ip in range(NP):
        x = np.array([px[:, ip]]).T
        w = pw[0, ip]

        #  Predict particle state with random input sampling
        ud1 = u[0, 0] + np.random.randn() * R_est[0, 0] ** 0.5
        ud2 = u[1, 0] + np.random.randn() * R_est[1, 1] ** 0.5
        ud = np.array([[ud1, ud2]]).T
        x = motion_model(x, ud, dt)

        #  Calc Importance Weight
        dx = x[0, 0] - z[0]
        dy = x[1, 0] - z[1]
        dz = math.hypot(dx, dy)
        w = w * gauss_likelihood(dz, math.sqrt(Q_est[0, 0]))

        px[:, ip] = x[:, 0]
        pw[0, ip] = w

    pw = pw / pw.sum()  # normalize

    x_est = px.dot(pw.T)
    p_est = calc_covariance(x_est, px, pw)

    N_eff = 1.0 / (pw.dot(pw.T))[0, 0]  # Effective particle number
    if N_eff < NTh:
        px, pw = re_sampling(px, pw)
    return x_est, p_est, px, pw


def re_sampling(px, pw):
    """
    low variance re-sampling
    """

    w_cum = np.cumsum(pw)
    base = np.arange(0.0, 1.0, 1 / NP)
    re_sample_id = base + np.random.uniform(0, 1 / NP)
    indexes = []
    ind = 0
    for ip in range(NP):
        while re_sample_id[ip] > w_cum[ind]:
            ind += 1
        indexes.append(ind)

    px = px[:, indexes]
    pw = np.zeros((1, NP)) + 1.0 / NP  # init weight

    return px, pw


def plot_covariance_ellipse(x_est, p_est):  # pragma: no cover
    p_xy = p_est[0:2, 0:2]
    eig_val, eig_vec = np.linalg.eig(p_xy)

    if eig_val[0] >= eig_val[1]:
        big_ind = 0
        small_ind = 1
    else:
        big_ind = 1
        small_ind = 0

    t = np.arange(0, 2 * math.pi + 0.1, 0.1)

    # eig_val[big_ind] or eiq_val[small_ind] were occasionally negative numbers extremely
    # close to 0 (~10^-20), catch these cases and set the respective variable to 0
    try:
        a = math.sqrt(eig_val[big_ind])
    except ValueError:
        a = 0

    try:
        b = math.sqrt(eig_val[small_ind])
    except ValueError:
        b = 0

    x = [a * math.cos(it) for it in t]
    y = [b * math.sin(it) for it in t]
    angle = math.atan2(eig_vec[big_ind, 1], eig_vec[big_ind, 0])
    Rot = np.array([[math.cos(angle), -math.sin(angle)],
                    [math.sin(angle), math.cos(angle)]])
    fx = Rot.dot(np.array([[x, y]]))
    px = np.array(fx[0, :] + x_est[0, 0]).flatten()
    py = np.array(fx[1, :] + x_est[1, 0]).flatten()
    plt.plot(px, py, "--r")


def main():

    dt = 0.1  # time tick [s]
    SIM_TIME = 50.0  # simulation time [s]
    time = 0.0

    show_animation = True

    # State Vector [x y yaw v]'
    x_est = np.zeros((4, 1))
    x_true = np.zeros((4, 1))

    # Particle states
    px = np.zeros((4, NP))  
    # Particle weights
    pw = np.zeros((1, NP)) + 1.0 / NP  

    # Dead reckoning state
    x_dr = np.zeros((4, 1))  

    # history
    h_x_est = x_est
    h_x_true = x_true
    h_x_dr = x_true

    while SIM_TIME >= time:
        time += dt
        u = calc_input()

        # z is exact distance to beacons
        x_true, z, x_dr, ud = observation(x_true, x_dr, u, dt)
        x_est, PEst, px, pw = pf_localization(px, pw, z, ud, dt)

        # store data history
        h_x_est = np.hstack((h_x_est, x_est))
        h_x_dr = np.hstack((h_x_dr, x_dr))
        h_x_true = np.hstack((h_x_true, x_true))


        # yaw/heading: true, estimated, dr
        print("yaw", time, x_true[2, 0], x_est[2, 0], x_dr[2, 0])

        if show_animation:
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect('key_release_event',
                                         lambda event: [exit(0) if event.key == 'escape' else None])

            plt.plot(px[0, :], px[1, :], ".r")
            plt.plot(np.array(h_x_true[0, :]).flatten(),
                     np.array(h_x_true[1, :]).flatten(), "-b")
            plt.plot(np.array(h_x_dr[0, :]).flatten(),
                     np.array(h_x_dr[1, :]).flatten(), "-k")
            plt.plot(np.array(h_x_est[0, :]).flatten(),
                     np.array(h_x_est[1, :]).flatten(), "-r")
            plot_covariance_ellipse(x_est, PEst)
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.001)


if __name__ == '__main__':
    main()
