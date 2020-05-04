import websockets
import asyncio
import logging

'''
A WebSocket end-point for the control of the robot, controlling the RPi and forwarding to the STM via a serial port.
'''

logger = logging.getLogger(__name__)

connections = set()
test = False


async def simulator():
    while True:
        await asyncio.sleep(1)
        await broadcast("Revs 0 0")
        await broadcast("t 0")
        await broadcast("r 0")


async def broadcast(message):
    if connections:
        await asyncio.gather([c.send(message) for c in connections])


async def connection(ws, path):
    logger.info("Connection from %r %r", ws, path)
    connections.add(ws)
    try:
        # async for message in ws:
        while True:
            message = await ws.recv()
            logger.debug("RECV %r" % message)
    except:
        logger.error("Connection error", exc_info=1)
    finally:
        connections.remove(ws)
    logger.info("Connection closed")


if __name__ == '__main__':

    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', default='', help='Bind to host address')
    parser.add_argument('-p', '--port', type=int, default=8000, help='Bind to port')
    parser.add_argument('-v', '--verbose', action='store_true', help="Verbose output")
    args = parser.parse_args()
    logging.basicConfig(level=args.verbose and logging.DEBUG or logging.INFO)

    server = websockets.serve(connection, args.host, args.port)
    asyncio.get_event_loop().run_until_complete(server)
    asyncio.get_event_loop().run_forever()