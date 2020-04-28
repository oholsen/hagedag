from typing import Optional
import json


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


class Revs(object):
    def __init__(self, segments):
        self.right = to_float(segments[1])
        self.left = to_float(segments[2])


class Speed(object):
    def __init__(self, segments):
        self.left = to_float(segments[1])
        self.right = to_float(segments[2])
        self.left_set_lp = to_float(segments[3])
        self.right_set_lp = to_float(segments[4])


class Power(object):
    def __init__(self, segments):
        self.left = to_float(segments[1])
        self.right = to_float(segments[2])
        self.left_I = to_float(segments[3])
        self.right_I = to_float(segments[4])


class Ignore:
    def __init__(self, segments):
        self.segments = segments

sentences = {
    "Revs": Revs,
    "Speed": Speed,
    "Power": Power,
}


def process(line):
    line = line.strip()
    segments = line.split()
    cmd = segments[0]
    # sentence = sentences.get(cmd, Record)
    sentence = sentences.get(cmd)
    if sentence:
        return sentence(segments)
