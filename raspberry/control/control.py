import serial
import pigpio
import time
import servo
import signal





pi = pigpio.pi()
pi.set_mode(14, pigpio.ALT5) # TXD
pi.set_mode(15, pigpio.ALT5) # RXD

stm = serial.Serial("/dev/ttyS0", baudrate=115200, timeout=3.0)
cutter = servo.cutter
cutter.power(0)


def handler(signum, frame):
    print('Heartbeat timeout with signal', signum)
    cutter.off()

# Set the signal handler 
signal.signal(signal.SIGALRM, handler)


def handle(data):
    # Dispatch message to RPI or STM
    data = data.strip()

    # TODO: Watchdog on the cutter, if loose touch then stop...
    if data.startswith('heartbeat'):
        # reset timer, pass on to STM
        signal.alarm(5) # seconds

    elif data == '!' or data == '.':
        cutter.off()
    
    elif data.startswith('CUT'):
        try:
            cols = data.split()
            cutter.power(float(cols[1]))
        except:
            cutter.off()
            print("Failed to handle CUT: " + data)
        return

    stm.write(data + '\n')

