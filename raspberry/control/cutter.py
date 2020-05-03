import pigpio

In1 = 23
In2 = 22

pi = pigpio.pi()
pi.set_mode(In1, pigpio.OUTPUT)
pi.set_mode(In2, pigpio.OUTPUT)


def off():
        pi.write(In1, 0)
        pi.write(In2, 0)


def forward():
        pi.write(In2, 0)
        pi.write(In1, 1)


def backward():
        pi.write(In1, 0)
        pi.write(In2, 1)


if __name__ == '__main__':
        import time
        off()
        raise SystemExit
        while True:
                forward()
                time.sleep(1)
                backward()
                time.sleep(1)
