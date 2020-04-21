import matplotlib.pyplot as plt
from math import cos, sin
from state import State


class Plot():
    def __init__(self):
        self.x = []
        self.y = []
        self.speed = []
        self.theta = []

    def show(self):
        plt.show()

    def update(self, state: State):
        self.plot(state.x, state.y, state.speed, state.theta)

    def plot(self, x, y, speed, theta):
        self.x.append(x)
        self.y.append(y)
        self.speed.append(speed)
        self.theta.append(theta)
        plt.cla()
        a = 5  # * speed
        plt.arrow(x, y, a * cos(theta), a * sin(theta))
        plt.axis("equal")
        plt.grid(True)
        plt.xlim(-25, 25)
        plt.ylim(-25, 25)
        plt.plot(self.x, self.y, "ob", label="trajectory", zorder=1)
        plt.pause(1e-9)
