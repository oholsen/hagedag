from dataclasses import dataclass


@dataclass
class State:
    x: float
    y: float
    theta: float
    speed: float
    omega: float

