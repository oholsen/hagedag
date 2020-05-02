import websockets
import asyncio
import aiostream
import logging
from datetime import datetime
import GPS
import RobotState
import play


logger = logging.getLogger()


async def from_robot(host):
    uri = f"ws://{host}:8000/control"
    while True:
        try:
            logging.info("Connecting to robot")
            async with websockets.connect(uri) as websocket:
                logger.info("Connected to robot")
                while True:
                    # await websocket.send("Hello world!")
                    msg = await websocket.recv()
                    msg = msg.strip()
                    logger.info("ROBOT %s", msg)
                    yield datetime.utcnow(), RobotState.process(msg)
        except:
            logger.error("From robot failed", exc_info=True)
        await asyncio.sleep(1)


async def from_gps(host):
    while True:
        try:
            # TODO: use write too - or in another connection???
            logging.info("Connecting to GPS")
            reader, _ = await asyncio.open_connection(host, 5000)
            logger.info("Connected to GPS")
            while True:
                msg = await reader.readline()
                msg = msg.decode().strip()
                logger.info("GPS %s", msg)
                yield datetime.utcnow(), GPS.process(msg)
        except:
            logger.error("From GPS failed", exc_info=True)
        await asyncio.sleep(1)


async def stream(host):
    async with aiostream.stream.merge(from_gps(host), from_robot(host)).stream() as streamer:
        async for m in streamer:
            yield m


async def record(host):
    logger.info("Connecting to host %s", host)
    try:
        # aiostream.stream.merge(from_gps(host), from_robot(host)) | aiostream.pipe.print()
        async for m in stream(host):
            print(m)
    except:
        logger.error("Record failed", exc_info=1)


async def track(host, yaw=0, speed=0):
    await play.track(stream(host), yaw, speed)


if __name__ == "__main__":
    import sys
    logging.basicConfig(
        level=logging.INFO,
        #filename="record.log",
        format='%(asctime)s %(levelname)s %(message)s',
    )
    host = sys.argv[1]
    #asyncio.run(record(host))
    asyncio.run(track(host))
