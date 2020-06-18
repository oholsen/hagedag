# import sys
import websockets
import asyncio
import logging


"""
A WebSocket end-point for high level mission control of the robot.
To be companied by a JS GUI.

Reports state from the robot:
- position, heading - to plot on map
- current target/mission
- state: current draw, etc, running, paused, waiting for GPS, ...
- GPS accuracy
- alarms: robot stuck, cutter stopped, out of fence, etc.

Accepts input:
- start/stop/pause mission
- helping controls: (manual, restart cutter, ....)
- change cutter speed, motor speed, ...
- Estop
- heartbeat

"""


logger = logging.getLogger(__name__)

connections = set()
test = False
#test = True


async def broadcast(message):
    if connections:
        await asyncio.wait([c.send(message) for c in connections])


class TestStream(asyncio.StreamReader):
    _i = 0

    async def readline(self):
        await asyncio.sleep(1)
        message = "Test message %d\n" % TestStream._i
        TestStream._i += 1
        return message.encode('utf-8')


async def read_control(reader):
    while True:
        line = await reader.readline()
        line = line.decode('utf-8')
        logger.debug("Read: %r", line)
        await broadcast(line)


async def handle(ws, path, writer):
    logger.info("Connection open from %r %r", ws, path)
    connections.add(ws)
    try:
        async for message in ws:
            logger.debug("RECV %r" % message)
            if message is None:
                message = ''
            message = str(message).strip()
    except:
        logger.exception("Connection error")
    finally:
        connections.remove(ws)
    logger.info("Connection closed")


async def main():
    reader = TestStream()
    rc = asyncio.create_task(read_control(reader))

    headers = websockets.http.Headers()
    headers["Access-Control-Allow-Origin"] = "*"

    logger.info("Starting server on %s:%d" % (args.host, args.port))
    await websockets.serve(lambda ws, path: handle(ws, path, writer),
                           args.host, args.port,
                           extra_headers=headers)


if __name__ == '__main__':

    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', default='', help='Bind to host address')
    parser.add_argument('-p', '--port', type=int, default=8000, help='Bind to port')
    parser.add_argument('-v', '--verbose', action='store_true', help="Verbose output")
    args = parser.parse_args()

    logging.basicConfig(level=args.verbose and logging.DEBUG or logging.INFO)

    asyncio.get_event_loop().run_until_complete(main())
    asyncio.get_event_loop().run_forever()
