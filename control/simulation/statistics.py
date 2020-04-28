import numpy as np
import math


def leastsq(x, y):
    n = len(x)
    assert len(y) == n
    X = np.sum(x)
    Y = np.sum(y)
    X2 = np.sum(x*x)
    XY = np.sum(x*y)
    Y2 = np.sum(y*y)
    d = n*X2 - X*X
    q = d * (n*Y2 - Y*Y)
    if d <= 0 or q <= 0: 
        return None
    # Y = ax + b
    ad = n*XY - X*Y
    bd = Y*X2 - X*XY
    r = ad / math.sqrt(q)
    return ad / d, bd / d, r


if __name__ == "__main__":
    xs = np.array([1, 2, 3])
    ys = np.array([1.1, 2.11, 3.09])
    print(leastsq(xs, ys))
