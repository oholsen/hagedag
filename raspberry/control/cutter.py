import pigpio


pi = pigpio.pi()
In1 = 23
In2 = 22
CutterPin = 23


class BrushedCutter:
        # No power regulation, geared motor is too slow at full speed
        def __init__(self):
                pi.set_mode(In1, pigpio.OUTPUT)
                pi.set_mode(In2, pigpio.OUTPUT)

        def off(self):
                pi.write(In1, 0)
                pi.write(In2, 0)

        def stop(self):
                self.off()

        def power(self, power):
                assert abs(power) <= 100
                if power >= 0:
                        # forward
                        pi.write(In2, 0)
                        pi.write(In1, 1)
                else:
                        # backward
                        pi.write(In1, 0)
                        pi.write(In2, 1)



class BrushlessCutter:
        # Forward only (airplane) motor
        def __init__(self):
                from servo import Servo
                self.cutter = Servo(CutterPin)

        def off(self):
                self.cutter.off()
                pi.write(In1, 0)
                pi.write(In2, 0)

        def stop(self):
                self.cutter.power(0)

        def power(self, power):
                self.cutter.power(power)


if __name__ == '__main__':
        import sys
        cutter = BrushlessCutter()
        try:
                cutter.power(float(sys.argv[1]))
        except:
                cutter.off()
                raise
