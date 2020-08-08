from typing import Optional
import json
import math
from abc import ABC

WHEEL_BASE         = 0.3300 # m
# max tick speed is about 5800
MAX_TICK_SPEED     = 5500
DIST_PER_TICK      = 1.145 / 20550 # m
MAX_SPEED          = MAX_TICK_SPEED * DIST_PER_TICK # m/s
# MAX_SPEED          = 0.15 # m/s, probably slightly higher, but to guarantee heading...

"""
Calibration on office floor:
20550 ticks
114.5 cm
"""


def to_float(s: str) -> Optional[float]:
    try:
        return float(s)
    except ValueError:
        return None

def to_int(s: str) -> Optional[int]:
    try:
        return int(s)
    except ValueError:
        return None

class FromRobot(ABC):
    pass

class Ticks(FromRobot):
    def __init__(self, segments):
        self.right = to_float(segments[1]) * DIST_PER_TICK
        self.left = to_float(segments[2]) * DIST_PER_TICK

    def __str__(self):
        return f"Revs({self.left}, {self.right})"

class Speed(FromRobot):
    def __init__(self, segments):
        self.left = to_float(segments[1]) * DIST_PER_TICK
        self.right = to_float(segments[2]) * DIST_PER_TICK
        self.left_set_lp = to_float(segments[3]) * DIST_PER_TICK
        self.right_set_lp = to_float(segments[4]) * DIST_PER_TICK

    def speed(self):
        return 1.5 * (self.left + self.right)

    def omega(self):
        return -2 * (self.left - self.right) / WHEEL_BASE

    def __str__(self):
        return f"Speed({self.left}, {self.right})"


class Power(FromRobot):
    def __init__(self, segments):
        self.left = to_float(segments[1])
        self.right = to_float(segments[2])
        self.left_I = to_float(segments[3])
        self.right_I = to_float(segments[4])


class Battery(FromRobot):
    def __init__(self, segments):
        self.voltage = float(segments[1])

    def __str__(self):
        return f"Battery({self.voltage})"


class Status(FromRobot):
    def __init__(self, segments):
        self.status = int(segments[1])

    def __str__(self):
        return f"Status({self.status})"

    def overload(self) -> bool:
        return self.status & 1 != 0


class Translate(FromRobot):
    def __init__(self, tick_speed: float):
        self.speed = tick_speed * DIST_PER_TICK

    def __str__(self):
        return f"Translate({self.speed})"


class Rotate(FromRobot):
    def __init__(self, tick_speed: float):
        # tick_speed is added and subtracted on motors
        self.omega = tick_speed * DIST_PER_TICK / (WHEEL_BASE / 2)

    def __str__(self):
        return f"Rotate({self.omega})"


class Stop(FromRobot):
    def __init__(self):
        pass
    def __str__(self):
        return "Stop()"

class Reset(Stop):
    def __init__(self):
        pass
    def __str__(self):
        return "Reset()"

class Heartbeat(FromRobot):
    def __str__(self):
        return "Heartbeat()"

def Control(segments):
    # print("Control", segments)
    cmd = segments[1]
    if cmd == '.':
        return Stop()
    if cmd == '!':
        return Reset()
    if cmd == 'r':
        return Rotate(float(segments[2]))
    if cmd == 't':
        return Translate(float(segments[2]))


class Ignore(FromRobot):
    def __init__(self, segments):
        self.segments = segments
    def __str__(self):
        return f"Ignore({self.segments})"


_sentences = {
    "Ticks": Ticks,
    "Speed": Speed,
    "Power": Power,
    "Control": Control,
    "Battery": Battery,
    "Status": Status,
    # "heartbeat": Heartbeat,
}


def process(line):
    line = line.strip()
    segments = line.split()
    cmd = segments[0]
    sentence = _sentences.get(cmd, Ignore)
    if sentence:
        return sentence(segments)


class RobotCommand(ABC):
    pass


class SpeedCommand(RobotCommand):
    def __init__(self, speed: float):
        self.speed = speed  # m/s
    def __str__(self):
        t = int(self.speed / DIST_PER_TICK) # ticks/s
        return f"t {t:.2f}"

class OmegaCommand(RobotCommand):
    def __init__(self, omega: float):
        self.omega = omega  # rad/s anti-clockwise from above
    def __str__(self):
        # tick_speed is added and subtracted on motors
        r = self.omega * (WHEEL_BASE / 2) /DIST_PER_TICK # ticks/s
        return f"r {r:.2f}"

class StopCommand(RobotCommand):
    def __str__(self):
        return "."

class ResetCommand(StopCommand):
    def __str__(self):
        return "!"

class HeartbeatCommand(RobotCommand):
    def __str__(self):
        return "heartbeat"

class CutCommand(RobotCommand):
    def __init__(self, speed: float):
        self.speed = speed  # just < 0, 0, > 0 for now
    def __str__(self):
        return f"CUT {self.speed:.2f}"
