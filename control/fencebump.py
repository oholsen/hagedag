import time
import logging
import asyncio
import random

from RobotState import StopCommand, SpeedCommand, OmegaCommand, CutCommand
import jsonobject
from shapely.geometry import Point, Polygon


logger = logging.getLogger(__name__)
returning: asyncio.Task = None


def load(filename):
    config = jsonobject.load(open(filename))
    points = list(((p.x, p.y) for p in config.limit))
    return Polygon(points)  # .buffer(-0.2, resolution=1, join_style=2)


async def bump2(x, y, fence, send, stop: asyncio.Event):
    p = Point(x, y)
    await bump(fence.contains(p), send, stop)


async def bump(inside, send, stop: asyncio.Event):
    logger.info("Bump %s", inside)
    global returning
    if stop.is_set():
        return
    if returning is not None and not returning.done():
        return
    if inside:
        await send(SpeedCommand(0.1))
        await send(CutCommand(15))
    else:
        returning = asyncio.create_task(avoid(0.1, 0.3, send, stop))


async def schedule(do, plan, stop: asyncio.Event):
    logger.debug("Starting schedule")
    for cmd, dt in plan:
        logger.debug("Schedule %s %.1f", cmd, dt)
        await do(cmd)
        if dt > 0:
            #await asyncio.sleep(dt)
            try:
                await asyncio.wait_for(stop.wait(), dt)
            except asyncio.TimeoutError:
                pass
            else:
                logger.info("Schedule stopped")
                return
    logger.debug("Schedule completed")


async def avoid(speed, turn, send, stop: asyncio.Event):
    logger.info("Starting avoidance")
    plan = [
        # reverse
        (SpeedCommand(-speed), 5 + 3 * random.random()), (SpeedCommand(0), 0),
        # turn
        (OmegaCommand(turn), 4 + 2 * random.random()), (OmegaCommand(0), 0),
        # go forward in main control
        ]        
    await schedule(send, plan, stop)


async def main():

    fence = load("garden.yaml")
    stop = asyncio.Event()
    async def send(x): logger.info("Send %s", x)

    for _ in range(2):
        await asyncio.sleep(1)
        await bump(True, send, stop)

    for _ in range(3):
        await asyncio.sleep(1)
        await bump(False, send, stop)

    for _ in range(3):
        await asyncio.sleep(1)
        await bump(True, send, stop)

    if 0:
        logging.info("Stopping!")
        stop.set()

    for _ in range(30):
        await asyncio.sleep(1)
        await bump(True, send, stop)


if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    asyncio.run(main())