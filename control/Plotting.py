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
        a = 1  # * speed
        plt.arrow(x, y, a * cos(theta), a * sin(theta))
        plt.axis("equal")
        plt.grid(True)
        plt.xlim(-10, 10)
        plt.ylim(-5, 5)
        plt.plot(self.x, self.y, ".b", label="trajectory", zorder=1)
        plt.pause(1e-9)
