import serial_asyncio
import pigpio
import servo
import signal
import logging
import asyncio

logger = logging.getLogger("controld.control")


pi = pigpio.pi()
pi.set_mode(14, pigpio.ALT5)  # TXD
pi.set_mode(15, pigpio.ALT5)  # RXD

# brushed motor
import cutter


if 0:
    # BLDC motor
    cutter = servo.cutter
    # Depending on whether motor driver (ESC) is reversible or not
    # cut = cutter.power # non-reversible
    cut = cutter.amplitude  # reversible





async def open():

    # https://tinkering.xyz/async-serial/
    reader, writer = await serial_asyncio.open_serial_connection(url="/dev/ttyS0", baudrate=115200, timeout=3.0)

    loop = asyncio.get_event_loop()
    loop.add_signal_handler(signal.SIGALRM, lambda: asyncio.ensure_future(timeout(writer)))

    # Arm the motor controller by giving PWM signal. for both reversible and non-reversible motors:
    # cut(0)
    cutter.off()

    return reader, writer


def send(writer, msg):
    msg = msg + '\n'
    writer.write(msg.encode('ascii'))
    

def stop(writer):
    # cut(0)
    cutter.off()
    send(writer, '.')


def timeout(writer):
    # also heartbeat timeout inside STM
    logger.info('Heartbeat timeout')
    stop(writer)


def handle(writer, data):
    # Dispatch message to RPI or STM
    assert data == data.strip()
    logger.debug("handle %r", data)

    if data == 'STOP':
        stop()
        return

    if data.startswith('CUT'):
        try:
            cols = data.split()
            speed = float(cols[1])
            if speed > 0:
                cutter.forward()
            elif speed < 0:
                cutter.backward()
            else:
                cutter.off()
        except:
            cutter.off()
            logger.error("Failed to handle CUT: %r", data, exc_info=1)
        return

    # Forward commands below to STM but intercept for local processing

    # Watchdog on the cutter, if loose touch then stop.
    if data.startswith('heartbeat'):
        # reset timer, pass on to STM
        signal.alarm(5)  # seconds
    elif data == '!':
        cut(0)
    elif data == '.':
        cutter.off()

    send(writer, data)
