import websockets
import asyncio
import logging

logger = logging.getLogger()
host = "192.168.1.139"


async def from_robot():
    uri = f"ws://{host}:8000/control"
    async with websockets.connect(uri) as websocket:
        logger.info("Connected to robot")
        while True:
            # await websocket.send("Hello world!")
            msg = await websocket.recv()
            msg = msg.strip()
            logger.info("ROBOT %s", msg)


async def from_gps():
    try:
        reader, _ = await asyncio.open_connection(host, 5000)
        logger.info("Connected to GPS")
        while True:
            msg = await reader.readline()
            msg = msg.decode().strip()
            logger.info("GPS %s", msg)
    except:
        logger.fatal("From GPS failed", exc_info=1)


async def main():
    logger.info("Connecting to host %s", host)
    await asyncio.gather(from_gps(), from_robot())


if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        filename="record.log",
        format='%(asctime)s %(levelname)s %(message)s',
    )
    asyncio.run(main())
