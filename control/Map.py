import numpy as np
from matplotlib.path import Path
from matplotlib.patches import PathPatch, Circle
import matplotlib.pyplot as plt
import jsonobject
from shapely.geometry import Point, Polygon, box
from shapely.ops import nearest_points


def polygon_pathpatch(points, **kwargs):
    codes = [Path.MOVETO] + [Path.LINETO] * (len(points) - 1) + [Path.CLOSEPOLY]
    vertices = points + [(0, 0)]
    vertices = np.array(vertices, float)
    # return vertices, codes
    path = Path(vertices, codes)
    pathpatch = PathPatch(path, **kwargs)
    return pathpatch

def shape_to_patch(shape, **kwargs):
    return polygon_pathpatch(list(shape.exterior.coords)[1:], **kwargs)    


def load_garden(filename) -> Polygon:
    config = jsonobject.load(open(filename))
    fence = Polygon(list(((p.x, p.y) for p in config.limit)))
    return fence


def load_mission(filename) -> jsonobject:
    config = jsonobject.load(open(filename))
    return config

def aoi_from_config(config):
    if config.aoi:
        p0 = config.aoi[0]
        p1 = config.aoi[1]
        return box(p0.x, p0.y, p1.x, p1.y)


class Config:

    def __init__(self, filename):
        self.mission = load_mission(filename)
        self.fence = load_garden(self.mission.garden)
        self.aoi = aoi_from_config(self.mission)
        self.centroid = self.fence.buffer(-0.5)


def main():
  
    fig, ax = plt.subplots()
    fig.set_size_inches(20, 8)

    mission = load_mission("mission.yaml")
    fence = load_garden(mission.garden)
    aoi = aoi_from_config(mission)

    ax.add_patch(shape_to_patch(fence, facecolor='None', edgecolor='red'))
    centroid = fence.buffer(-0.5)
    ax.add_patch(shape_to_patch(centroid, facecolor='khaki'))
   
    fence_new = load_garden("garden-new.yaml")
    ax.add_patch(shape_to_patch(fence_new, facecolor='None', edgecolor='yellow'))

    if aoi:
        print("fence", fence)
        print("x", fence.intersection(aoi))
        ax.add_patch(shape_to_patch(fence.intersection(aoi), facecolor='none', edgecolor='blue'))
        ax.add_patch(shape_to_patch(fence_new.intersection(aoi).buffer(-0.5), facecolor='darkkhaki'))

    p = Point(4.5, -6.6)
    ax.add_patch(Circle((p.x, p.y), 0.2))

    pc, _ = nearest_points(centroid, p)
    ax.add_patch(Circle((pc.x, pc.y), 0.2))
    # Reduce the centroid slightly to find direction, then just centroid for testing if it is inside

    ax.set_aspect('equal', 'box')
    ax.grid(True)
    # ax.set_title('A compound path')
    ax.autoscale_view()
    plt.show()


if __name__ == "__main__":
    main()