import logging
import RobotModel
import RobotMessages
import GPS
from datetime import datetime
from typing import Optional
import numpy as np
import math, cmath
import aiofiles
import state

logger = logging.getLogger(__name__)


def parse_time(line: str) -> datetime:
    t = line[:23].replace(",", ".")
    return datetime.fromisoformat(t)


def process(line: str) -> Optional[object]:
    # date time level module ROBOT/GPS ...
    try:
        line = line.strip()
        t = parse_time(line)
        cols = line.split()
        # print("cols", cols)
        if len(cols) < 5:
            return
        cols = cols[4:]
        cmd = cols[0]
        if cmd == "GPS" and len(cols) >= 2:
            return t, GPS.process(cols[1])

        if cmd == "ROBOT":
            # TODO: avoid joining
            msg = " ".join(cols[1:])
            return t, RobotState.process(msg)

        return t, None

    except:
        logger.exception("Parsing " + line)
        return None, None



async def parse(filename):
    async with aiofiles.open(filename) as f:
        async for line in f:
            yield process(line)


class RobotState(object):

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

        self.pos_time: datetime = None
        
        
        self.speeds_time: datetime = None
        self.speeds: RobotMessages.Speed = None        

        self.battery_ok = True
        self.battery: RobotMessages.Battery = None

        self.power_ok = True
        self.power: RobotMessages.Power = None

        # STM32 time
        self.time_time: datetime = None
        self.time: float = None

        self.status_time: datetime = None
        self.status: RobotMessages.Status = None


    def status_ok(self, t: datetime) -> bool:
        logger.debug("Status %r %r %r", self.status_time, self.status, t)
        if self.status is None:
            return False
        if abs((t - self.status_time).total_seconds()) > 3:
            return False
        return self.battery_ok and not self.status.max_power()


    def update(self, tt, o): # -> Optional[Tuple[t,dt,z,u]]
        # each GPS cycle starts with RMC, Revs are on same cycle - could interpolate and get speed???
        # yield tracker input state on each RMC
        # logger.debug("Feed update %s %r %s", tt, o, o)
        if tt is None or o is None:
            logger.error("Invalid input: %s %s", tt, o)

        if isinstance(o, GPS.RMC):
            # logger.debug("RMC %s %s %g", o.mode, o.has_rtk(), o.hdop)
            return

        if isinstance(o, GPS.GGA):
            # FIXME: use error in estimated position in tracking!!!
            # if o.hdop > 0.20 and not o.has_rtk(): return

            u = GPS.UTM(o.lat, o.lon)
            # print(o.time, o.lat, o.lon, o.alt)
            x = u.easting - GPS.u0.easting
            y = u.northing - GPS.u0.northing
            # FIXME: make georef configurable: u0 and rotation
            # Rotate by 20 degrees to align x,y coordinate system with garden.
            # Also rotates the heading/yaw reference system!
            # Y runs against house, X runs against hill
            p = complex(x, y)
            p = p * cmath.rect(1, math.radians(-20))
            x = p.real
            y = p.imag
            z = np.array([[x], [y]])

            speed, omega = self.speed1, self.omega1
            
            # TODO: use GPS time to discover network delay - need some magic around midnight
            if self.pos_time is None:
                dt = None
            else:
                dt = (tt - self.pos_time).total_seconds()
            self.pos_time = tt
            if dt is None:
                return

            ud = np.array([[speed], [omega]])
            # feed tracking.track()
            return tt, dt, z, ud, o.hdop

        if isinstance(o, RobotMessages.Time):
            self.time_time = tt
            self.time = o.time
            return

        if isinstance(o, RobotMessages.Speed):
            # print("SPEEDS", o)
            self.speeds_time = tt
            self.speeds = o
            self.speed2 = o.speed()
            self.omega2 = o.omega()
            #print("SPEEDS", self.speed2, self.omega2)
            return

        if isinstance(o, RobotMessages.StopAck):
            # print("STOP")
            self.speed1 = 0
            self.omega1 = 0
            return

        if isinstance(o, RobotMessages.MoveAck):
            # print("TRANS", o)
            self.speed1 = o.speed
            self.omega1 = o.omega
            return


        if isinstance(o, RobotMessages.Battery):
            logger.debug("BATTERY %s", o)
            self.battery = o
            # TODO: configurable threshold - eg global config "dep injection"
            if self.battery.voltage < 10:
                self.battery_ok = False
            return

        if isinstance(o, RobotMessages.Power):
            self.power = o
            # TODO: configurable threshold
            # TODO: continue (reset power flag - after a while? but could be stuck, need GUI...)
            # if self.power.max() > 0.8:
            #     self.power_ok = False
            return

        if isinstance(o, RobotMessages.Status):
            self.status = o
            self.status_time = tt
            # self.power_ok = self.power_ok and not self.status.max_power()
            # self.power_ok = not self.status.max_power()
