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



class BrushlessCutterUnidirectional:
        # Forward only (airplane) motor
        def __init__(self):
                from servo import Servo
                self.cutter = Servo(CutterPin)

        def off(self):
                self.cutter.off()

        def stop(self):
                self.cutter.power(0)

        def power(self, power):
                self.cutter.power(power)


class BrushlessCutterBidirectional:
        # Car/boat with reverse
        def __init__(self):
                from servo import Servo
                self.cutter = Servo(CutterPin)

        def off(self):
                self.cutter.off()

        def stop(self):
                self.cutter.power(0)

        def power(self, power):
                self.cutter.amplitude(power)


cutter = BrushlessCutterUnidirectional()


if __name__ == '__main__':
        import sys
        try:
                cutter.power(float(sys.argv[1]))
        except:
                cutter.off()
                raise
