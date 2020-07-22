from typing import Optional
import math
from abc import ABC


WHEEL_BASE         = 0.3300 # m
DIST_PER_WHEEL_REV = 0.3864 # m
MAX_SPEED          = 0.15 # m/s, probably slightly higher, but to guarantee heading...

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


class Time(FromRobot):
    def __init__(self, segments):
        self.time = to_float(segments[1])

    def __str__(self):
        return f"Time({self.time})"


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

    def max(self) -> float:
        return max(abs(self.left), abs(self.right))


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


class ControlAck(FromRobot):
    """Ack as response from starting processing of control line"""
    pass


class TranslateAck(ControlAck):
    def __init__(self, segments):
        # argument is [-20..20]
        self.speed = to_float(segments[1]) * 0.01  # m/s

    def __str__(self):
        return f"Translate({self.speed})"


class RotateAck(ControlAck):
    def __init__(self, segments):
        # argument is [-20..20]
        self.omega = -1 * math.radians(to_float(segments[1]))

    def __str__(self):
        return f"Rotate({self.omega})"


class HeartbeatAck(ControlAck):
    def __init__(self, segments):
        pass

    def __str__(self):
        return "Heartbeat()"


class StopAck(ControlAck):
    def __init__(self, segments):
        pass

    def __str__(self):
        return "Stop()"


class ResetAck(ControlAck):
    def __init__(self, segments):
        pass

    def __str__(self):
        return "Reset()"


_control_acks = {
    "t": TranslateAck,
    "r": RotateAck,
    ".": StopAck,
    "!": ResetAck,
    "heartbeat": HeartbeatAck,
}


def control_ack(segments) -> Optional[ControlAck]:
    assert segments[0] == "Control"
    segments = segments[1:]
    ack = _control_acks.get(segments[0])
    if ack is not None:
        return ack(segments)
    return None


class Ack(FromRobot):
    # Ack <time> <command as interpreted by robot>
    def __init__(self, time: float, command: FromRobot):
        self.time = time
        self.command = command

    def __str__(self):
        return f"Ack({self.time},{self.command})"


class Move(FromRobot):
    def __init__(self, segments):
        # speed is [-20..20] # cm/s
        # omega is [-20..20] # degrees/s
        self.speed = 1 * int(to_float(segments[1])) * 0.01 # m/s
        self.omega = -1 * math.radians(to_float(segments[2]))
        self.timeout = to_float(segments[3])

    def __str__(self):
        return f"Move({self.speed},{self.omega},{self.timeout})"


class Timeout(FromRobot):
    # Timeout <time> <command as interpreted by robot>
    def __init__(self, time: float, command: FromRobot):
        self.time = time
        self.command = command

    def __str__(self):
        return f"Timeout({self.time},{self.command})"


_commands = {
    "m": Move,
}


def command(clz, segments) -> FromRobot:
    time = float(segments[1])
    segments = segments[2:]
    cmd = _commands.get(segments[0])
    if cmd is not None:
        cmd = cmd(segments)
        return clz(time, cmd)
    return None


class Ignore(FromRobot):
    def __init__(self, segments):
        self.segments = segments

    def __str__(self):
        return f"Ignore({self.segments})"


_sentences = {
    "Time": Time,
    "Revs": Revs,
    "Speed": Speed,
    "Power": Power,
    "Battery": Battery,
    "Control": control_ack,
    "Ack": lambda segments: command(Ack, segments),
    "Timeout": lambda segments: command(Timeout, segments),
}


def process(line):
    line = line.strip()
    segments = line.split()
    cmd = segments[0]
    sentence = _sentences.get(cmd, Ignore)
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


class MoveCommand(RobotCommand):
    def __init__(self, speed: float, omega: float, timeout: float):
        self.speed = speed  # m/s
        self.omega = omega  # rad/s
        self.timeout = timeout

    def __str__(self):
        t = 100 * self.speed  # cm/s
        r = - math.degrees(self.omega)  # degrees/s
        return f"m {t:.2f} {r:.2f} {self.timeout:.3f}"


class StopCommand(RobotCommand):
    def __str__(self):
        return "."


class ResetCommand(StopCommand):
    def __str__(self):
        return "!"


class CutCommand(RobotCommand):
    def __init__(self, speed: float):
        self.speed = speed

    def __str__(self):
        return f"CUT {self.speed:.2f}"
