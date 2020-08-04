import asyncio
import math
from math import sin, cos, pi
import cmath
from dataclasses import dataclass, replace
import logging
import time
import random
import RobotMessages
import RobotState
from state import State
from PID import PID
from Plotting import Plot
from abc import ABC
from shapely.geometry import Point, Polygon, MultiPolygon, LineString, JOIN_STYLE
from shapely.ops import nearest_points


log = logging.getLogger(__name__)


class Control(ABC):

    def update(self, t: float, state: State): # -> (speed, omega)
        return None, None

    def end(self, t: float, state) -> bool:
        return False


class GetStateControl(Control):
    """Terminates immediately just to get current state"""

    def update(self, t: float, state: State): # -> (speed, omega)
        assert False

    def end(self, t: float, state) -> bool:
        return True


class CompositeControl2(Control):
    """Use sequence of controls, end() on the current control triggers using next control"""

    def __init__(self, controls):
        self.controls = iter(controls)
        self.control = next(self.controls)
        log.info("Starting %s", self.control)

    def update(self, t: float, state: State): # -> (speed, omega)
        assert self.control is not None
        # log.debug("Composite update %s %f %s", self.control, t, state)
        while self.control.end(t, state):
            # log.debug("Composite end %s %f %s", self.control, t, state)
            try:
                self.control = self.controls.send((t, state))
            except (StopAsyncIteration, StopIteration):
                log.info("Stopped CompositeControl2")
                self.control = None
                return None, None
            log.info("Starting %s %s", t, self.control)
        return self.control.update(t, state)

    def end(self, t: float, state) -> bool:
        return self.control is None


class StraightControl(Control):

    def __init__(self, speed: float, end):
        self._end = end
        self.speed = speed

    def __str__(self):
        return f"StraightControl({self.speed})"

    def update(self, t: float, state: State): # -> (speed, omega)
        # state vectors [x y yaw v]'
        return self.speed, 0

    def end(self, t: float, state: State) -> bool:
        return self._end(t, state)


def norm_angle(a: float) -> float:
    while a > pi:
        a -= 2 * pi
    while a <= -pi:
        a += 2 * pi
    return a


class HLineControl(Control):

    def __init__(self, y: float, right: bool, end_x: float, speed=0.2, omega=0.2):
        self.pid = PID(0.4, 1, 1.5, 0.2)
        self.y = y
        self.right = right
        self.end_x = end_x
        self.speed = speed
        self.omega = omega
        self.t_last = None

    def __str__(self):
        return f"HLineControl({self.right},{self.end_x})"

    def reset(self):
        self.pid.clear()

    def update(self, t: float, state: State): # -> (speed, omega)
        d = self.y - state.y
        # angle is 90 for large d
        theta = (pi/2) * (1 - math.exp(-(d/0.1)**2))
        if d < 0:
            theta = -theta
        if not self.right:
            theta = pi - theta
        dtheta = norm_angle(theta - state.theta)
        # reduce speed if theta is very wrong, 1 at 0, 0.2 at pi/2
        # also reduce speed if will overshoot in one iteration:
        #   sin(theta) * speed * 1s <= abs(d)
        #  speed <= abs(d) / sin(theta)
        speed = self.speed * math.exp(-abs(4*dtheta)**2)
        if math.sin(theta) > 0:
            speed = min(speed, abs(d) / math.sin(theta) + 0.02)
        omega = math.copysign(min(abs(dtheta) + 0.02, self.omega), dtheta)
        domega = 0

        
        if False and self.t_last is not None and abs(d) < 0.4:
            # only use PID if near the line
            # TODO: remove this discontinuity
            domega = -self.pid.update(d, t - self.t_last)
            if self.right:
                domega = -domega
        else:
            # TODO: reset PID? or update PID without using result?
            pass
        #print(t, d, theta, state.theta, dtheta, speed, domega)
        self.t_last = t
        return speed, omega + domega


    def end(self, t: float, state: State) -> bool:
        return self.right == (state.x >= self.end_x)



class LineControl(Control):

    def __init__(self, p0, p1, speed, omega):
        self.line = LineString([p0, p1])
        self.p0 = complex(*p0)
        self.p1 = complex(*p1)
        self.dp = self.p1 - self.p0
        self.p2 = self.p1 + self.dp # p2 is p0 mirrored around p1
        self.theta = cmath.phase(self.dp)
        self.speed = speed
        self.omega = omega
        self._end = nearest(self.p0.real, self.p0.imag, self.p2.real, self.p2.imag)

    def __str__(self):
        return f"LineControl({self.line})"

    def update(self, t: float, state: State): # -> (speed, omega)
        p = complex(state.x, state.y)

        # cross product: gives distance and which side
        dp = p - self.p0
        d = dp.real * self.dp.imag - dp.imag * self.dp.real
        d /= abs(self.dp)

        # angle wrt line. dtheta is 90 for large d
        dtheta = (pi/2) * (1 - math.exp(-(d/0.2)**2))

        # limit overshoot - could account for omega too,
        # find arc to hit line 
        speed = math.inf
        if math.sin(dtheta) > 0:
            speed = abs(d) / math.sin(dtheta) + 0.02

        # which side is it on????
        if d < 0:
            dtheta = -dtheta
        dtheta = norm_angle(self.theta + dtheta - state.theta)
        # print("theta", self.theta, state.theta, dtheta)
        # reduce speed if theta is very wrong, 1 at 0, 0.2 at pi/2
        speed = min(speed, self.speed * math.exp(-abs(4*dtheta)**2))
        
        # also reduce speed to not overshoot crossing line
        d1 = abs(p - self.p1)
        if d1 < 2 * state.speed:
            speed = min(d1/4 + 0.02, speed)

        omega = math.copysign(min(abs(dtheta), self.omega), dtheta)
        # print(t, d, theta, state.theta, dtheta, speed)
        return speed, omega

    def end(self, t: float, state: State) -> bool:
        #return False
        return self._end(t, state)


class PointControl(Control):

    def __init__(self, x: float, y: float, speed: float, omega: float, end):
        self.x = x
        self.y = y
        self.speed = speed
        self.omega = omega
        self._end = end

    def __str__(self):
        return f"PointControl({self.x},{self.y})"

    def update(self, t: float, state: State): # -> (speed, omega)
        dx = self.x - state.x
        dy = self.y - state.y
        theta = math.atan2(dy, dx)
        dtheta = norm_angle(theta - state.theta)
        # reduce speed if theta is very wrong, 1 at 0, 0.2 at pi/2
        speed = self.speed * math.exp(-abs(dtheta/0.1)**2)
        # reduce speed near point since 1s update rate can overshoot
        d = math.hypot(dx, dy)
        if d < 2 * speed: # two seconds
            speed = min(speed, d/3 + 0.05)
        # relax towards desired _theta
        omega = math.copysign(min(abs(dtheta), self.omega), dtheta)
        return speed, omega

    def end(self, t: float, state: State) -> bool:
        return self._end(t, state)


def distance(x, y, r):
    def d(t: float, state: State):
        dx = x - state.x
        dy = y - state.y
        d = math.hypot(dx, dy)
        return d < r
    return d


def nearest(x0, y0, x1, y1):
    def end(t: float, state: State):
        d0 = math.hypot(x0 - state.x, y0 - state.y)
        d1 = math.hypot(x1 - state.x, y1 - state.y)
        return d1 <= d0
    return end


class TimeControl(Control):

    def __init__(self, speed: float, omega: float, time: float):
        self.speed = speed
        self.omega = omega
        self.time = time # seconds
        self.t0 = None

    def __str__(self):
        return f"TimeControl({self.time})"


    def update(self, t: float, state: State): # -> (speed, omega)
        if self.t0 is None:
            self.t0 = t
        return self.speed, self.omega

    def end(self, t: float, state: State) -> bool:
        return self.t0 is not None and t >= self.t0 + self.time



class TimeControl2(Control):

    def __init__(self, speed: float, omega: float, end_time: float):
        self.speed = speed
        self.omega = omega
        self.end_time = end_time

    def __str__(self):
        return f"TimeControl2"

    def update(self, t: float, state: State): # -> (speed, omega)
        return self.speed, self.omega

    def end(self, t: float, state: State) -> bool:
        return t >= self.end_time


def start_arc(radius, speed, direction):
    # speed may be too high for outer motor in tight turn
    # outer motor speed is speed + omega * wheelbase / 2
    # max_speed_of_motor = speed + speed * wheelbase / radius / 2
    speed = min(speed, RobotMessages.MAX_SPEED / (1 + 0.5 * RobotMessages.WHEEL_BASE / radius))
    omega = speed / radius
    if not direction:
        omega = -omega
    return speed, omega

    
class ArcControl(Control):

    def __init__(self, speed, omega, end_theta):
        self.end_theta = end_theta
        self.dtheta_last = None
        self.speed = speed
        self.omega = omega

    def __str__(self):
        return f"ArcControl({self.end_theta})"

    def update(self, t: float, state: State): # -> (speed, omega)
        # TODO: slow down (both!) near end state such that it just overshoots
        # zero crossing on the next update
        return self.speed, self.omega

    def end(self, t: float, state: State) -> bool:
        # stop at first zero crossing, but not a random jump +-pi
        dtheta = norm_angle(state.theta - self.end_theta)
        if self.dtheta_last is None:
            self.dtheta_last = dtheta
            return False

        if abs(dtheta) < 0.9:
            if self.dtheta_last < 0 and dtheta >= 0: return True
            if self.dtheta_last > 0 and dtheta <= 0: return True
        self.dtheta_last = dtheta
        return False


def LineTest(xl, xr, y0):
    end_x = {True: xr, False: xl}
    speed_max = 0.20
    omega_max = 0.20
    t, state = yield GetStateControl()
    right = state.x < (xl + xr) / 2    
    while True:
        t, state = yield HLineControl(y0, right, end_x[right])
        t, state = yield TimeControl(0, omega_max * random.uniform(-1, 1), t + 3)
        t, state = yield TimeControl(speed_max * random.uniform(0.1, 1), 0, t + 4)
        t, state = yield TimeControl(0, omega_max * random.uniform(-1, 1), t + 3)
        right = not right


def ScanHLine(x0, y0, x1, y1, speed, omega):
    assert x1 > x0
    assert y1 > y0
    dy = 0.14
    y = y0
    right = True
    end_x = {True: x1, False: x0}
    end_theta = {True: 0, False: pi}
    while True:
        # TODO: dummy first control, with end() just to get state?
        yield HLineControl(y, right, end_x[right])
        y += dy
        right = not right
        if y > y1: 
            y = y0
        else:
            # speed may be too high for outer motor in tight turn
            s, o = start_arc(dy/2, speed, right)
            yield ArcControl(s, -o, end_theta[right])


def end_inside(poly: Polygon):
    def f(t: float, state: State):
        return poly.contains(Point(state.x, state.y))
    return f


def end_outside(poly: Polygon):
    def f(t: float, state: State):
        return not poly.contains(Point(state.x, state.y))
    return f


def FenceBumps(fence, speed, omega):
    from Map import Point, nearest_points
    import random
    centroid = fence.buffer(-0.5)
    target = centroid.buffer(-0.2)
    while True:
        t, state = yield StraightControl(speed, end_outside(fence))
        pc, _ = nearest_points(target, Point(state.x, state.y))
        t, state = yield PointControl(pc.x, pc.y, speed, omega, end_inside(centroid))
        # random direction and duration - duration bias to scan more of area
        t, state = yield TimeControl2(0, random.choice((-omega, omega)), t + 4 + 4 * random.random()) 


def RingControls(coords, speed, omega):
    x0, y0 = coords[0]
    yield PointControl(x0, y0, speed, omega, distance(x0, y0, 0.2))
    # aim a little beyond the desired point to make sure it crosses the line before turning wildly to reach the point        
    ahead = 0.2 # meters
    for x, y in coords[1:]:
        # x1,y1 is x0,y0 mirrored around perpendicular at x,y - crossing perpendicular when closer to x1,y1
        dx = x - x0
        dy = y - y0
        x1 = x + dx
        y1 = y + dy
        d = math.hypot(dx, dy)
        yield PointControl(x + ahead * dx / d, y + ahead * dy / d, speed, omega, nearest(x0, y0, x1, y1))
        x0 = x
        y0 = y


def PathControls(coords, speed, omega):
    p0 = coords[0]
    for p in coords[1:]:
        yield LineControl(p0, p, speed, omega)
        p0 = p


def FenceShrink(limits, aoi, speed, omega):
    shrink = -0.15
    _, state = yield GetStateControl()

    area = 0
    shapes = [aoi]
    while shapes:
        # select the closest area:
        _, shape = min((s.distance(Point(state.x, state.y)), s) for s in shapes)
        shapes.remove(shape)
        lap = 1
        area += 1
        while True:
            if isinstance(shape, MultiPolygon):
                log.info("Fence %d MultiPolygon %d", area, len(shape.geoms))
                shapes.extend(shape.geoms)
                break

            log.info("Fence %d lap %d area %g", area, lap, shape.area)
            l = [(math.hypot(state.x - x, state.y - y), i, x, y) for i, (x, y) in enumerate(shape.exterior.coords)]
            l.sort()
            assert shape.exterior.coords[0] == shape.exterior.coords[-1]
            _, i = min((math.hypot(state.x - x, state.y - y), i) for i, (x, y) in enumerate(shape.exterior.coords[:-1]))

            # shuffle coordinates (rotate) to start with the nearest point
            coords = shape.exterior.coords[i:-1] + shape.exterior.coords[:i]
            coords.append(coords[0])
            assert len(coords) == len(shape.exterior.coords)

            import geometry
            path = geometry.path((state.x, state.y), coords[0], limits)
            if path is None:
                log.warning("Fence %d: no path to first point %s", area, coords[0])
                break

            # move to first point in ring
            for c in PathControls(path.coords, speed, omega):
                _, state = yield c
            
            # move around ring
            for c in PathControls(coords, speed, omega):
                _, state = yield c
            
            # make a smaller ring
            shape2 = shape.buffer(shrink, join_style=JOIN_STYLE.mitre)
            if shape2.area < 0.001:
                log.info("Fence %d almost completed: area %g", area, shape2.area)
                shape2 = shape.buffer(shrink/2, join_style=JOIN_STYLE.mitre)
                log.info("Fence %d completion: area %g", area, shape2.area)

            if shape2.area < 0.001:
                # also covers area==0 with no coords
                log.info("Fence %d completed: area %g", area, shape2.area)
                break
            shape = shape2
            lap += 1



def LineControlTest():
    speed = 0.1
    omega = 0.2
    yield PointControl(0, 0, speed, omega, distance(0, 0, 0.1))
    yield LineControl((0,0), (5,0), speed, omega)
    yield LineControl((5,0), (5,5), speed, omega)


# TODO: remove, move to record.py
async def simulate_point():
    from RobotModel import RobotModel
    model = RobotModel(State(1, 1, 0, 0))
    x = -5
    y = 2
    control = PointControl(x, y, 0.2, 0.2, distance(x, y, 0.4))
    plot = Plot()
    dt = 0.1
    t = 0.0
    state = model.get_state()
    plot.update(state)
    while not control.end(t, state):
        t += dt
        model.update(dt)
        state = model.get_state()
        plot.update(state)
        speed, omega = control.update(t, state)
        if speed is not None:
            model.set_speed_omega(speed, omega)


def main():
    asyncio.run(simulate_point())


if __name__ == "__main__":
    logging.basicConfig(
        level=logging.DEBUG,
        #filename="control.log",
        format='%(asctime)s %(levelname)s %(message)s',
    )
    main()
