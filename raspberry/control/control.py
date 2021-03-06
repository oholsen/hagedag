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

from cutter import cutter



def reset_timeout():
    signal.alarm(5)  # seconds


def timeout(writer):
    logger.info('Cutter timeout')
    cutter.stop()
    # stop(writer)


async def open():

    # https://tinkering.xyz/async-serial/
    reader, writer = await serial_asyncio.open_serial_connection(url="/dev/ttyS0", baudrate=115200, timeout=3.0)

    loop = asyncio.get_event_loop()
    # loop.add_signal_handler(signal.SIGALRM, lambda: asyncio.ensure_future(timeout(writer)))
    loop.add_signal_handler(signal.SIGALRM, lambda: timeout(writer))

    # Arm the motor controller by giving PWM signal. 
    # For both reversible and non-reversible motors:
    cutter.stop()

    return reader, writer


def send(writer, msg):
    msg = msg + '\n'
    writer.write(msg.encode('ascii'))
    

def stop(writer):
    send(writer, '.')


def handle(writer, data):
    # Dispatch message to RPI or STM
    assert data == data.strip()
    logger.debug("handle %r", data)

    if data == 'STOP':
        stop(writer)
        return

    if data.startswith('CUT'):
        try:
            cols = data.split()
            power = float(cols[1])
            cutter.power(power)
            reset_timeout()
        except:
            cutter.off()
            logger.error("Failed to handle CUT: %r", data, exc_info=1)
        return

    # Forward commands below to STM but intercept for local processing

    if data == '!':
        cutter.off()
    elif data == '.':
        pass
        # cutter.off()

    send(writer, data)
