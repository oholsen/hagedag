from dataclasses import dataclass
import numpy as np

@dataclass
class State:
    """State required to update state by system model"""
    # omega is not part a state!
    x: float
    y: float
    theta: float
    speed: float

    def to_array(self):
        return np.array([[self.x], [self.y], [self.theta], [self.speed]])


def from_array(s) -> State:
    s = s.flatten()
    return State(*s)
