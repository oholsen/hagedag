from typing import Optional
import json
import math
from abc import ABC


WHEEL_BASE         = 0.3300 # m
DIST_PER_WHEEL_REV = 0.3864 # m

"""
Calibration on office floor:
119 cm
3.08 revs (with 36 ticks per rev, R = 6cm)
38.64 cm / rev
37.68 = 2*6*3.14
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

class Revs(FromRobot):
    def __init__(self, segments):
        self.right = to_float(segments[1]) * DIST_PER_WHEEL_REV
        self.left = to_float(segments[2]) * DIST_PER_WHEEL_REV

    def __str__(self):
        return f"Revs({self.left}, {self.right})"

class Speed(FromRobot):
    def __init__(self, segments):
        self.left = to_float(segments[1]) * DIST_PER_WHEEL_REV
        self.right = to_float(segments[2]) * DIST_PER_WHEEL_REV
        self.left_set_lp = to_float(segments[3]) * DIST_PER_WHEEL_REV
        self.right_set_lp = to_float(segments[4]) * DIST_PER_WHEEL_REV

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



"""
float cmd_speed = 0.0f;    // forward revs / sec
float cmd_rotation = 0.0f; // right from above

#define ROTATION_DIST           (M_PI * WHEEL_BASE) // cm


cmd_speed from GUI

rotation from GUI
cmd_rotation = rotation * (ROTATION_DIST / 360.0); // deg/s -> cm/s


float speed_L = (cmd_speed + cmd_rotation) * TICKS_PER_CM;
float speed_R = (cmd_speed - cmd_rotation) * TICKS_PER_CM;


Tracking and control:
omega/theta is positive left from above

speed = cmd_speed # cm/s
omega = -rotation # deg/s
"""



class Translate(FromRobot):
    def __init__(self, speed: float):
        # argument is [-20..20]
        self.arg = speed
        self.speed = 1 * int(speed) * 0.01 # m/s

    def __str__(self):
        return f"Translate({self.speed})"

class Rotate(FromRobot):
    def __init__(self, omega: float):
        # argument is [-20..20]

        self.arg = omega
        self.omega = -1 * math.radians(omega)

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


_sentences = {
    "Revs": Revs,
    "Speed": Speed,
    "Power": Power,
    "Control": Control,
}


def process(line):
    line = line.strip()
    segments = line.split()
    cmd = segments[0]
    sentence = _sentences.get(cmd) # , Ignore)
    if sentence:
        return sentence(segments)


def revs_delta2(revs1: Revs, revs2: Revs, dt: float):
    dr = revs2.right - revs1.right
    dl = revs2.left - revs1.left
    return revs_delta(dl, dr, dt)


def revs_delta(dl: float, dr: float, dt: float):
    speed = 3 * 0.5 * (dl + dr) * DIST_PER_WHEEL_REV / dt # m/s
    omega = 2 * 0.5 * (dr - dl) / WHEEL_BASE / dt
    return speed, omega


class RobotCommand(ABC):
    pass

class SpeedCommand(RobotCommand):
    def __init__(self, speed: float):
        self.speed = speed  # m/s
    def __str__(self):
        t = 100 * self.speed  # cm/s
        return f"t {t:.2f}"

class OmegaCommand(RobotCommand):
    def __init__(self, omega: float):
        self.omega = omega  # omega is rad/s in opposite direction to 
    def __str__(self):
        r = - math.degrees(self.omega)  # degrees/s
        return f"r {r:.2f}"

class StopCommand(RobotCommand):
    def __str__(self):
        return "."

class ResetCommand(RobotCommand):
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
