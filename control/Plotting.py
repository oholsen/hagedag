from math import cos, sin
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.path import Path
from matplotlib.patches import PathPatch, Circle
from matplotlib import patches
from shapely.geometry import Point, Polygon, box, LineString, MultiLineString, MultiPolygon

from state import State



def polygon_path(points):
    #codes = [Path.MOVETO] + [Path.LINETO] * (len(points) - 1) + [Path.CLOSEPOLY]
    #vertices = points + [(0, 0)]
    codes = [Path.MOVETO] + [Path.LINETO] * (len(points) - 1)
    vertices = points
    vertices = np.array(vertices, float)
    return Path(vertices, codes, closed=True)


def shape_to_patch(shape, **kwargs):

    if isinstance(shape, Polygon):
        return patches.Polygon(list(shape.exterior.coords)[1:], **kwargs)

    if isinstance(shape, MultiPolygon):
        paths = [polygon_path(list(o.exterior.coords)[1:]) for o in shape.geoms]
        for p in paths:
            print("path", p)
        path = Path.make_compound_path(*paths)
        return PathPatch(path, **kwargs)

    if isinstance(shape, LineString):
        path_data = [(Path.MOVETO, shape.coords[0])]
        for coord in shape.coords[1:]:
            path_data.append((Path.LINETO, coord))
        codes, verts = zip(*path_data)
        path = Path(verts, codes, closed=False)
        return PathPatch(path, **kwargs)


def lines_to_patch(lines, **kwargs):
    path_data = []
    for line in lines:
        path_data.append((Path.MOVETO, line.coords[0]))
        path_data.append((Path.LINETO, line.coords[1]))
    codes, verts = zip(*path_data)
    path = Path(verts, codes)
    return PathPatch(path, **kwargs)



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
        self.state = None

    def show(self):
        self.render()
        plt.show()

    def pause(self):
        plt.pause(1e-9)

    def add_shape(self, shape, **kwargs):
        """Add shapely Shape outline"""
        patch = shape_to_patch(shape, **kwargs)
        self.patches.append(patch)

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
        if self.state:
            self.ax.add_patch(Circle((self.state.x, self.state.y), 0.3, zorder=2, facecolor="lightgrey", edgecolor="black"))
            a = 1  # * speed
            self.ax.arrow(self.state.x, self.state.y, a * cos(self.state.theta), a * sin(self.state.theta), zorder=2) # top
        self.ax.plot(self.x, self.y, ".b", label="trajectory", zorder=1) # bottom

        # GUI event loop
        plt.pause(0.001)
