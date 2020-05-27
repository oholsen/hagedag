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


def load(filename) -> Polygon:
    config = jsonobject.load(open(filename))
    points = list(((p.x, p.y) for p in config.limit))
    return Polygon(points)

def main():
    config = jsonobject.load(open("garden.yaml"))
    # print(config)
    points = list(((p.x, p.y) for p in config.limit))
    fence = Polygon(points)
  
    fig, ax = plt.subplots()
    fig.set_size_inches(20, 8)
    ax.add_patch(shape_to_patch(fence, facecolor='None', edgecolor='red'))

    centroid = fence.buffer(-1)#, resolution=1) #, join_style=2)
    ax.add_patch(shape_to_patch(centroid, facecolor='green', edgecolor='green'))

    aoi = box(10, -10, 20, 10)
    aoi = fence.intersection(aoi)
    ax.add_patch(shape_to_patch(aoi, facecolor='none', edgecolor='blue'))

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