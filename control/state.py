from dataclasses import dataclass


@dataclass
class State:
    """State required to update state by system model"""
    # omega is not part a state!
    x: float
    y: float
    theta: float
    speed: float
    # TODO: from/to array


def from_array(s) -> State:
    s = s.flatten()
    return State(*s)
