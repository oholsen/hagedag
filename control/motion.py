import numpy as np

class MotionFilter:

    def __init__(self, t, pos):
        self.last_t = t
        self.last_pos = pos
        self.v = np.array([0, 0])
        self.alpha = 0.1

    def update(self, t, pos):
        dt = t - self.last_t
        dpos = pos - self.last_pos
        v = dpos/dt
        self.v = (1 - self.alpha) * self.v + self.alpha * v
        self.last_t = t
        self.last_pos = pos

    def pos(self):
        return self.last_pos

    def velocity(self):
        return self.v


def main():

    f = None
    lines = open('recording/2019-09-10-184511.pts')
    lines.readline()
    for line in lines:
        cols = line.strip().split()
        _, frame, x, y = cols
        frame = int(frame)
        x = float(x)
        y = float(y)
        p = np.array([x, y])
        if f is None:
            f = MotionFilter(frame, p)
            continue

        f.update(frame, p)

        print(frame, x, y, "%.2f %.2f" % tuple(f.velocity().tolist()))


if __name__ == '__main__':
    main()
