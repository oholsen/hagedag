import numpy as np
from matplotlib.path import Path
from matplotlib.patches import PathPatch, Circle
from matplotlib import patches
import matplotlib.pyplot as plt
import jsonobject
from shapely.geometry import Point, Polygon, box, LineString, MultiLineString, MultiPolygon
from shapely.ops import nearest_points
import geometry


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
    print("Fence area", fence.area)
    ax.add_patch(shape_to_patch(fence, facecolor='khaki', edgecolor='black'))

    if 0:
        path = geometry.path((1,-8), (10,-6), fence)
        # path = geometry.path((-11,2), (-5,-1), fence)
        print("Path", path, path.length)
        ax.add_patch(shape_to_patch(path, facecolor='None', edgecolor='blue'))

    if 1:
        centroid = fence.buffer(-0.5)
        #centroid = fence.buffer(-1.5, join_style=2)
        ax.add_patch(shape_to_patch(centroid, facecolor='orange'))   
    
    fence_new = load_garden("garden-new.yaml")
    ax.add_patch(shape_to_patch(fence_new, facecolor='None', edgecolor='orange'))

    if aoi:
        print("AOI", aoi)
        print("FENCE", fence)
        aoi = fence.intersection(aoi)
        print("AOI", aoi)
        # print("fence", fence)
        # print("x", fence.intersection(aoi))
        # ax.add_patch(shape_to_patch(fence.intersection(aoi), facecolor='none', edgecolor='blue'))
        ax.add_patch(shape_to_patch(aoi, facecolor='darkkhaki', edgecolor="black"))

    if 0:
        ring = centroid.exterior
        print("centroid exterior", ring.length, len(ring.coords))
        p = Point(4.5, -6.6)
        ax.add_patch(Circle((p.x, p.y), 0.2))

        pc, _ = nearest_points(centroid, p)
        ax.add_patch(Circle((pc.x, pc.y), 0.2))
        # Reduce the centroid slightly to find direction, then just centroid for testing if it is inside

        p = Point(5.5, -6.6)
        pc, _ = nearest_points(ring, p)
        ax.add_patch(Circle((pc.x, pc.y), 0.2))


    if 0:
        x0 = -20
        x1 = 20
        y = -10
        dy = 0.14
        lines = []
        while y < 10:
            line = LineString([(x0, y), (x1, y)])
            line = line.intersection(fence)
            if not line.is_empty:
                if isinstance(line, MultiLineString):
                    lines.extend(line.geoms)
                else:
                    lines.append(line)
            y += dy
        # print(lines)
        # ax.add_patch(lines_to_patch(lines))

    ax.set_aspect('equal', 'box')
    ax.grid(True)
    # ax.set_title('Garden')
    ax.autoscale_view()
    plt.show()


if __name__ == "__main__":
    main()