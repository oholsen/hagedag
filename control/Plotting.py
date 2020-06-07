import numpy as np
import matplotlib.pyplot as plt
from math import cos, sin
from matplotlib.path import Path
from matplotlib.patches import PathPatch, Circle
from state import State


class Plot():
    
    def __init__(self, ax:plt.Axes=None, frames_per_plot=1):
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
        self.frames_per_plot = frames_per_plot

    def show(self):
        self.render()
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
        self.state = state        
        self.x.append(state.x)
        self.y.append(state.y)
        self.speed.append(state.speed)
        self.theta.append(state.theta)

        self.frame += 1
        if self.frame < self.frames_per_plot:
            return
        self.frame = 0
        self.render()

    def render(self):
        self.ax.cla()
        self.ax.set_aspect('equal', 'box')
        self.ax.grid(True)
        for p in self.patches:
            self.ax.add_patch(p)

        # ax.set_title('Garden Map')
        # self.ax.autoscale_view()
        self.ax.add_patch(Circle((self.state.x, self.state.y), 0.15))
        a = 1  # * speed
        self.ax.arrow(self.state.x, self.state.y, a * cos(self.state.theta), a * sin(self.state.theta), zorder=2) # top
        self.ax.plot(self.x, self.y, ".b", label="trajectory", zorder=1) # bottom

        # GUI event loop
        plt.pause(0.001)
