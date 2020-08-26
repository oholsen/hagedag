import jsonobject
from shapely.geometry import Point, Polygon, box, LineString, MultiLineString, MultiPolygon
from shapely.ops import nearest_points, linemerge
import math


def path(p0, p1, fence) -> LineString:

    # FIXME: support holes too - so fence is really an area/shape that is navigable
    coords = fence.exterior.coords
    
    # make sure the contains predicate is true also for points on the exterior.
    # alternatively, could shrink/erode area in order to get candidate corners!
    # anyway, better not to cut the corners too tight anyway. but risk that p0 or p1
    # is outside the eroded area.
    fence = fence.buffer(0.01)

    # check for direct path
    line = LineString([p0, p1])
    if fence.contains(line):
        return line

    paths = {} #  corner -> shortest path from p0
    queue = [] #  corners to be processed

    # init with direct line from p0 to every corner
    for c in coords:
        line = LineString([p0, c])
        if fence.contains(line):
            paths[c] = line
            queue.append(c)

    # find shortest path from p0 to every corner.
    # not every corner is directly accessible from p0.
    while queue:
        # we have already found a shortest path from p0 to c0.
        # check if c0 can be in the shortest path to other corners.
        c0 = queue.pop(0)
        for c in coords:
            if c == c0:
                continue
            line = LineString([c0, c])            
            if not fence.contains(line):
                continue
            # Two paths to c: paths[c] and paths[c0] + line(c0,c)
            path1a = paths.get(c)
            path1b = linemerge([paths[c0], line])
            if path1a is None or path1b.length < path1a.length:
                # path1b is shorter
                assert fence.contains(path1b)
                paths[c] = path1b
                # Need to check again if there is a shorter path to c?
                # There cannot possibly be one? The shortest path will be the fewest number of hops.
                # The shortest path will be in this iteration over c, so no need to enqueue?
                # queue.append(c)

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
    # p = Polygon([(0,0), (10,0), (10,10), (5, 10), (5,5), (0,5)])    
    # p = Polygon([(0,0), (1,0), (1,1), (0,1)])    
    print(p)
    l = LineString([(0,0), (1,0)])
    print(l)
    print(p.buffer(0.01).contains(l))

    #pp = path((1,4), (6,9), p)
    pp = path((1,9), (6,9), p)
    print("path", pp)