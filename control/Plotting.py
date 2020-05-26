import numpy as np
import matplotlib.pyplot as plt
from math import cos, sin
from matplotlib.path import Path
from matplotlib.patches import PathPatch, Circle
from state import State


class Plot():
    
    def __init__(self, ax:plt.Axes=None):
        self.x = []
        self.y = []
        self.speed = []
        self.theta = []
        if ax is None:
            fig, ax = plt.subplots()
            fig.set_size_inches(20, 8)
        self.ax = ax
        self.patches = []
        self.frame = 0
        self.frames_per_plot = 10

    def show(self):
        plt.show()

    def pause(self):
        plt.pause(1e-9)

    def add_shape(self, shape, **kwargs):
        """Add shapely Shape outline"""
        points = list(shape.exterior.coords)[1:]
        codes = [Path.MOVETO] + [Path.LINETO] * (len(points) - 1) + [Path.CLOSEPOLY]
        vertices = points + [(0, 0)]
        vertices = np.array(vertices, float)
        path = Path(vertices, codes)
        self.patches.append(PathPatch(path, **kwargs))

    def update(self, state: State):
        self.plot(state.x, state.y, state.speed, state.theta)

    def plot(self, x, y, speed, theta):
        self.x.append(x)
        self.y.append(y)
        self.speed.append(speed)
        self.theta.append(theta)

        self.frame += 1
        if self.frame < self.frames_per_plot:
            return
        self.frame = 0

        self.ax.cla()
        self.ax.set_aspect('equal', 'box')
        self.ax.grid(True)
        # ax.set_title('Garden Map')
        self.ax.autoscale_view()
        a = 1  # * speed
        self.ax.arrow(x, y, a * cos(theta), a * sin(theta), zorder=2) # top
        self.ax.plot(self.x, self.y, ".b", label="trajectory", zorder=1) # bottom

        for p in self.patches:
            self.ax.add_patch(p)

        plt.pause(1e-9)
        #plt.show()
