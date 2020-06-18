import jsonobject
from shapely.geometry import Point, Polygon, box, LineString, MultiLineString, MultiPolygon
from shapely.ops import nearest_points, linemerge
import math


def path(p0, p1, fence) -> LineString:
    coords = fence.exterior.coords
    fence = fence.buffer(0.01)
    line = LineString([p0, p1])
    if fence.contains(line):
        return line

    paths = {} #  coord -> shortest path from p0
    queue = []

    # lines from p0 to every corner
    for c in coords:
        line = LineString([p0, c])
        if fence.contains(line):
            paths[c] = line
            queue.append(c)

    # find shortest path to every corner
    while queue:
        c0 = queue.pop(0)
        for c in coords:
            if c == c0:
                continue
            line = LineString([c0, c])            
            if not fence.contains(line):
                continue
            path0 = paths[c0]
            path1 = paths.get(c)
            if path1 is None:
                path1b = linemerge([path0, line])
            else:
                # Two paths to c: path1 and path0 + line
                path1b = linemerge([path0, line])
                if path1b.length > path1.length:
                    path1b = None
            if path1b:
                # will enque while finding shorter paths
                assert fence.contains(path1b)
                paths[c] = path1b
                queue.append(c)

    # find shortest path to destination
    dist = math.inf
    path = None
    for c, p in paths.items():
        line = LineString([c, p1])
        p = linemerge([p, line])
        if not fence.contains(p):
            continue
        # print("short", p.length, p)
        if p.length < dist:
            path = p
            dist = p.length
    return path


if __name__ == "__main__":
    p = Polygon([(0,0), (10,0), (10,10), (5, 10), (5,5), (4,5), (4,10), (0,10)])    
    # L shape
    #p = Polygon([(0,0), (10,0), (10,10), (5, 10), (5,5), (0,5)])    
    # p = Polygon([(0,0), (1,0), (1,1), (0,1)])    
    print(p)
    l = LineString([(0,0), (1,0)])
    print(l)
    print(p.buffer(0.01).contains(l))

    #pp = path((1,4), (6,9), p)
    pp = path((1,9), (6,9), p)
    print("path", pp)