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


def process(line: str) -> Optional[object]:
    try:
        line = line.strip()
        cols = line.split()
        if len(cols) < 4:
            return

        t = line[:23].replace(",", ".")
        tt = datetime.fromisoformat(t)

        if len(cols) == 5 and cols[3] == "GPS":
            msg = cols[4]
            return tt, GPS.process(msg)

        if len(cols) > 5 and cols[3] == "ROBOT":
            msg = " ".join(cols[4:])
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

        self.rmc_time: datetime  = None
        self.rmc: GPS.RMC = None
        
        self.revs_time: datetime = None
        self.revs: RobotState.Revs = None
        
        self.speed_time: datetime = None
        self.speeds: RobotState.Speed = None        

    def update(self, tt, o):
        # each GPS cycle starts with RMC, Revs are on same cycle - could interpolate and get speed???
        # yield tracker input state on each RMC
        if tt is None or o is None:
            return
        if isinstance(o, GPS.RMC):
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
            #speed, omega = self.speed2, self.omega2
            #speed, omega = self.speed3, self.omega3
            
            if self.rmc_time is None:
                dt = None
            else:
                dt = (tt - self.rmc_time).total_seconds()
                #dx, dy = u.diff(GPS.UTM(self.rmc.lat, self.rmc.lon))
                #speed_gps = math.hypot(dx, dy) / dt
                #print("SPEEDGPS", dl/dt)

            ud = np.array([[speed], [omega]])
            # TODO: track time!?
            # print("SPEEDOMEGA", speed, omega)
            self.rmc_time = tt
            self.rmc = o
            return tt, dt, z, ud

        if isinstance(o, GPS.GGA):
            if self.rmc:
                assert o.lat == self.rmc.lat
                assert o.lon == self.rmc.lon
            return

        if isinstance(o, RobotState.Revs):
            #print("REVS", o)
            # Will be lagging somewhat, up to a second - could extrapolate from speeds when yielding at RMC!?
            if self.revs_time is not None:
                dt = (tt - self.revs_time).total_seconds()
                if dt > 0.1: # avoid divide by zero
                    self.speed3, self.omega3 = RobotState.revs_delta2(self.revs, o, dt)                    
                #print("REVS", self.speed3, self.omega3, dt)
            self.revs = o
            self.revs_time = tt
            return

        if isinstance(o, RobotState.Speed):
            # print("SPEEDS", o)
            self.speeds_time = tt
            self.speeds = o
            self.speed2 = o.speed()
            self.omega2 = o.omega()
            #print("SPEEDS", self.speed2, self.omega2)
            return

        if isinstance(o, RobotState.Stop):
            print("STOP")
            self.speed1 = 0
            self.omega1 = 0
            return

        if isinstance(o, RobotState.Translate):
            print("TRANS", o)
            self.translate = o
            self.speed1 = o.speed
            return

        if isinstance(o, RobotState.Rotate):
            print("ROT", o)
            self.rotate = o
            self.omega1 = o.omega
            return



async def track_input(stream):
    p = RobotStateFeed()
    async for t, o in stream:
        if t is not None and o is not None:
            # t, dt, z, u or None
            m = p.update(t, o)
            if m is not None:
                yield m


async def track(stream, yaw=0, speed=0):
    # async for x in track_input(stream): print("track", repr(x))
    await tracking.track(track_input(stream), yaw=yaw, speed=speed)


if __name__ == "__main__":
    import asyncio
    logging.basicConfig(
        level=logging.INFO,
        filename="record.log",
        format='%(asctime)s %(levelname)s %(message)s',
    )
    filename = sys.argv[1]
    stream = parse(filename)
    asyncio.run(track(stream, yaw=-0.80*math.pi))