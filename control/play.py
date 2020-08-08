import logging
import sys
import RobotState
import GPS
from datetime import datetime
from typing import Optional
import numpy as np
import tracking
import math, cmath
import aiofiles
import re
import state

logger = logging.getLogger(__name__)


def process(line: str) -> Optional[object]:
    try:
        line = line.strip()
        cols = re.split(r' +', line)
        if len(cols) < 4:
            return

        t = line[:23].replace(",", ".")
        tt = datetime.fromisoformat(t)

        if len(cols) == 6 and cols[4] == "GPS":
            msg = cols[5]
            return tt, GPS.process(msg)

        if len(cols) > 5 and cols[4] == "ROBOT":
            msg = " ".join(cols[5:])
            return tt, RobotState.process(msg)

        return tt, None
    except:
        return None, None


async def parse(filename):
    async with aiofiles.open(filename) as f:
        async for line in f:
            yield process(line)


class RobotStateFeed(object):

    def __init__(self, speed:float=0, omega:float=0):
        # Input (u) [speed omega]' from Speed, commands, or Revs.
        # Revs seems to slow down without any good reason (GPS progresses with same speed).
        # Speed is more noisy but can have aliasing (LP not slow enough).
        
        # Commands
        self.speed1 = speed
        self.omega1 = omega

        # Speeds
        self.speed2 = speed
        self.omega2 = omega

        # Revs
        self.speed3 = speed
        self.omega3 = omega

        self.pos_time: datetime  = None
                
        self.speed_time: datetime = None
        self.speeds: RobotState.Speed = None        

        self.battery_ok = True
        self.battery: RobotState.Battery = None

    # TODO: also report error state - or be able to return status from here...

    def update(self, tt, o): # -> Optional[Tuple[t,dt,z,u]]
        # each GPS cycle starts with RMC, Revs are on same cycle - could interpolate and get speed???
        # yield tracker input state on each RMC

        # logger.debug("Feed update %s %r %s", tt, o, o)

        if tt is None or o is None:
            logger.error("Invalid input: %s %s", tt, o)
            return

        if isinstance(o, GPS.RMC):
            # logger.debug("RMC %s %s %g", o.mode, o.has_rtk(), o.hdop)
            return

        if isinstance(o, GPS.GGA):
            # FIXME: use error in estimated position in tracking!!!
            # if o.hdop > 0.20 and not o.has_rtk(): return

            u = GPS.UTM(o.lat, o.lon)
            # print(o.time, o.lat, o.lon, o.alt)
            deasting = u.easting - GPS.u0.easting
            dnorthing = u.northing - GPS.u0.northing
            x = deasting
            y = dnorthing
            # Rotate by 20 degrees to align x,y coordinate system with garden.
            # Also rotates the heading/yaw reference system!
            # Y runs against house, X runs against hill
            p = complex(x, y)
            p = p * cmath.rect(1, math.radians(-20))
            x = p.real
            y = p.imag
            z = np.array([[x], [y]])

            speed, omega = self.speed1, self.omega1
            
            # TODO: use tt or GPS time? Can track dt outside.
            if self.pos_time is None:
                dt = None
            else:
                dt = (tt - self.pos_time).total_seconds()
                #dx, dy = u.diff(GPS.UTM(self.rmc.lat, self.rmc.lon))
                #speed_gps = math.hypot(dx, dy) / dt
                #print("SPEEDGPS", dl/dt)

            ud = np.array([[speed], [omega]])
            self.pos_time = tt
            return tt, dt, z, ud, o.hdop

        if isinstance(o, RobotState.Speed):
            # print("SPEEDS", o)
            self.speeds_time = tt
            self.speeds = o
            self.speed2 = o.speed()
            self.omega2 = o.omega()
            #print("SPEEDS", self.speed2, self.omega2)
            return

        if isinstance(o, RobotState.Stop):
            # print("STOP")
            self.speed1 = 0
            self.omega1 = 0
            return

        if isinstance(o, RobotState.Translate):
            # print("TRANS", o)
            self.translate = o
            self.speed1 = o.speed
            return

        if isinstance(o, RobotState.Rotate):
            # print("ROT", o)
            self.rotate = o
            self.omega1 = o.omega
            return

        if isinstance(o, RobotState.Battery):
            logger.debug("BATTERY %s", o)
            self.battery = o
            # TODO: configurable threshold - eg global config "dep injection"
            if self.battery.voltage < 10:
                self.battery_ok = False
            return


async def track_input(stream):
    # TODO: drops time
    p = RobotStateFeed()
    async for s in stream:
        if s is None: continue
        t, o = s
        # logger.debug("track input {%s} {%s}", t, o)
        if t is None or o is None:
            continue

        # returns (t, dt, z, u) or None
        m = p.update(t, o)
        # logger.debug("track input %s %r -> %r", t, o, m)
        if not p.battery_ok:
            logger.debug("Battery not ok: %r %s", p.battery_ok, p.battery)
            continue
        if m is not None:
            # logger.debug("track input %s %r -> %r", t, o, m)
            yield m


async def track(stream, yaw=0, speed=0):
    # async for x in track_input(stream): print("track", repr(x))
    # return await tracking.track(track_input(stream), yaw=yaw, speed=speed)
    async for s in tracking.track(track_input(stream), yaw=yaw, speed=speed):
        s = state.from_array(s)
        logger.debug("STATE %g %g %g %g", s.x, s.y, s.theta, s.speed)
        yield s
