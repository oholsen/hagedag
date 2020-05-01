import websockets
import asyncio
import logging

logger = logging.getLogger()


async def from_robot(host):
    uri = f"ws://{host}:8000/control"
    while True:
        try:
            async with websockets.connect(uri) as websocket:
                logger.info("Connected to robot")
                while True:
                    # await websocket.send("Hello world!")
                    msg = await websocket.recv()
                    msg = msg.strip()
                    logger.info("ROBOT %s", msg)
        except:
            logger.error("From robot failed", exc_info=1)
        await asyncio.sleep(1)


async def from_gps(host):
    while True:
        try:
            reader, _ = await asyncio.open_connection(host, 5000)
            logger.info("Connected to GPS")
            while True:
                msg = await reader.readline()
                msg = msg.decode().strip()
                logger.info("GPS %s", msg)
        except:
            logger.error("From GPS failed", exc_info=1)
        await asyncio.sleep(1)


async def main():
    host = "192.168.1.139"
    logger.info("Connecting to host %s", host)
    try:
        await asyncio.gather(from_gps(host), from_robot(host))
    except:
        logger.error("Record failed", exc_info=1)


if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        filename="record.log",
        format='%(asctime)s %(levelname)s %(message)s',
    )
    asyncio.run(main())
