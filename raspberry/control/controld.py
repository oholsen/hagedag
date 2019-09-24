# import sys
import websockets
import asyncio
import logging

'''
A WebSocket end-point for the control of the robot, controlling the RPi and forwarding to the STM via a serial port.
'''

logger = logging.getLogger("controld.main")

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
        message = "From STM %d\n" % TestStream._i
        TestStream._i += 1
        return message.encode('ascii')


async def read_control(reader):
    while True:
        line = await reader.readline()
        line = line.decode('ascii')
        logger.debug("From STM: %r", line)
        await broadcast(line)


async def handle(ws, path, writer):
    # print(dir(ws))
    logger.info("Connection from %r %r", ws, path)
    connections.add(ws)
    try:
        # async for message in ws:
        while True:
            message = await ws.recv()
            logger.debug("RECV %r" % message)
            if message is None:
                message = ''
            message = str(message).strip()
            if not test:
                import control
                control.handle(writer, message)
    except:
        logger.error("Connection error", exc_info=1)
    finally:
        connections.remove(ws)
    logger.info("Connection closed")


async def main():
    if test:
        reader, writer = TestStream(), None
    else:
        import control
        reader, writer = await control.open()

    asyncio.ensure_future(read_control(reader))

    headers = websockets.http.Headers()
    headers["Access-Control-Allow-Origin"] = "*"

    logger.info("Starting controld on %s:%d" % (args.host, args.port))
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
