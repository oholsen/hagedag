import time
from abc import ABC

class ControlTime(ABC):
    def tick(self) -> None:
        pass
    def time(self) -> float:
        pass
    def dt(self) -> float:
        pass


class RealTime(ControlTime):
    def __init__(self):
        self.t = time.time()
        self._last_dt = None

    def tick(self) -> None:
        t = time.time()
        self._last_dt = t - self.t
        self.t = t

    def time(self):
        return self.t

    def dt(self) -> float:
        return self._last_dt


class SimulatedTime(ControlTime):
    def __init__(self, dt: float):        
        self.t = 0
        self._dt = dt

    def tick(self) -> None:
        self.t += self._dt

    def time(self):
        return self.t

    def dt(self) -> float:
        return self._dt


control_time = SimulatedTime(0.1)
