import websockets
import asyncio
import aiostream
import logging
import time
from datetime import datetime
import GPS
import RobotState
import play
import control
from state import State

logger = logging.getLogger(__name__)


async def controld_reader(websocket, incoming):
    while True:
        msg = await websocket.recv()
        t = datetime.utcnow()
        msg = msg.strip()
        logger.debug("ROBOT %s", msg)
        await incoming.put((t, RobotState.process(msg)))
        # incoming.put_nowait((t, RobotState.process(msg)))


async def controld_writer(websocket, outgoing):
    while True:
        tm, m = await outgoing.get() # also time to make sure not outputting old messages
        t = datetime.utcnow()
        if (t - tm).total_seconds() > 3:
            logger.warning("Old controld message %s %s", tm, m)
            continue
        m = str(m) 
        logger.debug("Sending controld: %s", m)
        await websocket.send(m + "\n")
        outgoing.task_done()


async def controld_connection(host, incoming, outgoing):
    uri = f"ws://{host}:8000/control"
    while True:
        try:
            logger.info("Connecting to controld")
            async with websockets.connect(uri) as websocket:
                logger.info("Connected to controld")
                consumer_task = asyncio.create_task(controld_reader(websocket, incoming))
                producer_task = asyncio.create_task(controld_writer(websocket, outgoing))
                _, pending = await asyncio.wait(
                    [consumer_task, producer_task],
                    return_when=asyncio.FIRST_COMPLETED
                )
                for task in pending:
                    task.cancel()                
        except:
            logger.error("Controld connection failed", exc_info=True)
        await asyncio.sleep(1)


async def from_controld(host):
    uri = f"ws://{host}:8000/control"
    while True:
        try:
            logger.info("Connecting to controld")
            async with websockets.connect(uri) as websocket:
                logger.info("Connected to controld")
                while True:
                    # await websocket.send("Hello world!")
                    msg = await websocket.recv()
                    msg = msg.strip()
                    logger.info("ROBOT %s", msg)
                    yield datetime.utcnow(), RobotState.process(msg)
        except:
            logger.error("From controld failed", exc_info=True)
        await asyncio.sleep(1)


async def gps_consumer(reader, incoming):
    while True:
        try:
            msg = await reader.readline()
            t = datetime.utcnow()
            msg = msg.decode().strip()
            logger.debug("GPS %s", msg)
            msg = GPS.process(msg)
            if msg:
                await incoming.put((t, msg))
        except:
            logger.error("GPS decode", exc_info=True)
            raise


async def from_gps(host):
    while True:
        try:
            # TODO: use write too - or in another connection???
            logger.info("Connecting to GPS")
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


async def gps_connection(host, incoming: asyncio.Queue):
    import re
    log_filter = re.compile(r".G.(RMC|GGA|VTG).*")
    while True:
        try:
            logger.info("Connecting to GPS")
            reader, _ = await asyncio.open_connection(host, 5000)
            logger.info("Connected to GPS")
            # await asyncio.create_task(gps_consumer(reader, incoming))
            while True:
                msg = await reader.readline()
                t = datetime.utcnow()
                msg = msg.decode().strip()
                if log_filter.match(msg):
                    logger.debug("GPS %s", msg)
                msg = GPS.process(msg)
                if msg: 
                    await incoming.put((t, msg))
        except:
            logger.error("GPS connection", exc_info=True)
        await asyncio.sleep(2)


async def incoming_consumer(incoming: asyncio.Queue):
    while True:
        t, m = await incoming.get()
        print("INCOMING", t, m)
        incoming.task_done()


async def heartbeat(outgoing):
    while True:
        await asyncio.sleep(2)
        t = datetime.utcnow()
        m = RobotState.HeartbeatCommand()
        await outgoing.put((t, m))


# also serves to record - without the outgoing producer
async def test_streamq(host):
    incoming = asyncio.Queue()
    outgoing = asyncio.Queue()
    ti = asyncio.create_task(incoming_consumer(incoming))
    to = asyncio.create_task(outgoing_producer(outgoing))
    tc = asyncio.create_task(controld_connection(host, incoming, outgoing))
    tg = asyncio.create_task(gps_connection(host, incoming))
    await asyncio.gather(tc, tg, ti, to)


# could take stream of controld commands as input!?
# creat a task in from_controld that consumes the stream
# but can we consume from different tasks???
async def stream(host):
    async with aiostream.stream.merge(from_gps(host), from_controld(host)).stream() as streamer:
        async for m in streamer:
            yield m


async def record(host):
    logger.info("Connecting to host %s", host)
    try:
        # aiostream.stream.merge(from_gps(host), from_robot(host)) | aiostream.pipe.print()
        async for m in stream(host):
            pass # print(m)
    except:
        logger.error("Record failed", exc_info=1)



async def incoming_generator(incoming):
    while True:
        t, m = await incoming.get()
        # logger.debug("incoming %r %r", t, m)
        yield t, m
        incoming.task_done()


async def track(host, yaw=0, speed=0):
    await play.track(stream(host), yaw, speed)


async def track2(host, yaw=0, speed=0):
    incoming = asyncio.Queue()
    outgoing = asyncio.Queue()
    # ti = asyncio.create_task(incoming_consumer(incoming))
    # to = asyncio.create_task(heartbeat(outgoing))
    tc = asyncio.create_task(controld_connection(host, incoming, outgoing))
    tg = asyncio.create_task(gps_connection(host, incoming))
    # await asyncio.gather(tc, tg, ti, to)
    # incoming -> track -> control -> outgoing
    async for s in play.track(incoming_generator(incoming), yaw, speed):
        logger.info("track2 %g %g %g %g", s.x, s.y, s.theta, s.speed)


async def line_control(host, yaw=0, speed=0):
    incoming = asyncio.Queue()
    outgoing = asyncio.Queue()
    tc = asyncio.create_task(controld_connection(host, incoming, outgoing))
    tg = asyncio.create_task(gps_connection(host, incoming))
    to = asyncio.create_task(heartbeat(outgoing))

    # TODO: wait for first observation to get x0, y0
    x0 = 0
    y0 = -3

    y = -3
    xl = -9
    xr = 10
    _control = control.CompositeControl(control.LineTest(x0, y0, xl, xr, y))
    # plot = Plot()

    # incoming -> track -> control -> outgoing
    async for t, state in play.track(incoming_generator(incoming), yaw, speed):
        # t = time.time()
        # plot.update(state)
        speed, omega = _control.update(t, state)
        if speed is not None:
            t = datetime.utcnow()
            m = RobotState.SpeedCommand(speed)
            await outgoing.put((t, m))
        if omega is not None:
            t = datetime.utcnow()
            m = RobotState.OmegaCommand(omega)
            await outgoing.put((t, m))


def simulated_control(control, plot):
    from RobotModel import RobotModel
    model = RobotModel(State(0, 0, 0, 0))
    dt = 1
    t = 0.0
    state = model.get_state()
    plot.update(state)
    while not control.end(t, state):
        t += dt
        state = model.update(dt)
        plot.update(state)
        speed, omega = control.update(t, state)
        if speed is not None and omega is not None:
            model.set_speed_omega(speed, omega)


async def realtime_control(host, control, plot):
    incoming = asyncio.Queue()
    outgoing = asyncio.Queue()
    # Keep references to tasks to avoid them being stopped instantly
    tc = asyncio.create_task(controld_connection(host, incoming, outgoing))
    tg = asyncio.create_task(gps_connection(host, incoming))
    # to = asyncio.create_task(heartbeat(outgoing))

    # incoming -> track -> control -> outgoing
    async for state in play.track(incoming_generator(incoming), 0, 0):
        t = time.time()
        plot.update(state)
        # continue
        speed, omega = control.update(t, state)
        # logger.debug("Update %s -> %r %r", state, speed, omega)
        if speed is not None:
            t = datetime.utcnow()
            m = RobotState.SpeedCommand(speed)
            await outgoing.put((t, m))
        if omega is not None:
            t = datetime.utcnow()
            m = RobotState.OmegaCommand(omega)
            await outgoing.put((t, m))
        if speed is not None and omega is not None:
            t = datetime.utcnow()
            m = RobotState.CutCommand(20)
            await outgoing.put((t, m))
            # TODO: timeout on commands
            t = datetime.utcnow()
            m = RobotState.HeartbeatCommand()
            await outgoing.put((t, m))


def replay(file, plot):
    i = 0
    for line in file:
        t = line[:23]
        line = line.strip()
        cols = line.split()
        if len(cols) >= 9 and cols[4] == "STATE":
            state = State(*map(float, cols[5:]))
            print(t, state)
            plot.update(state)
            i = 0
        else:
            i += 1
            if i > 1000:
                print(line)
                i = 0


async def mission_control(host, filename):
    from Map import Config
    from Plotting import Plot
    from control import CompositeControl2, ScanHLine, FenceBumps, RingControls, FenceShrink

    config = Config("mission.yaml")
    if config.aoi:
        fence = config.fence.intersection(config.aoi)
    else:
        fence = config.fence
    speed = config.mission.speed or 0.05
    omega = config.mission.omega or 0.2

    # controls = FenceBumps(fence, speed, omega)
    # controls = RingControls(fence.exterior.coords, speed, omega)
    # controls = FenceShrink(fence, speed, omega)
    # controls = FenceShrink(fence.buffer(-0.2, join_style=2), speed, omega)
    # controls = ScanHLine(-10, -4.5, 13, -1.5, speed, omega) # midten
    # controls = ScanHLine(-10.5, -1.6, 17, -0.9, speed, omega) # roser
    # controls = ScanHLine(-10, -7.5, -1, -4, speed, omega) # slackline - gml gran
    # controls = ScanHLine(-10, -9, -6, -4, speed, omega) # slackline - paere

    controls = ScanHLine(-2, -4, 2, -1, speed, omega) # test

    control = CompositeControl2(controls)
    logger.info("Starting mission control for %s", controls)

    plot = Plot(frames_per_plot=host is None and 50 or 1)
    plot.add_shape(config.fence, facecolor="khaki") # Entire grass
    plot.add_shape(fence, facecolor="darkkhaki") # AOI
    plot.pause()

    if host is None:
        if filename:
            replay(open(filename), plot)
        else:
            simulated_control(control, plot)
    else:
        await realtime_control(host, control, plot)
    plot.pause()
    plot.show()

if __name__ == "__main__":
    import sys, yaml
    from os import path
    import logging.config
    with open("record.yaml") as f:
        logging.config.dictConfig(yaml.full_load(f))
    arg = len(sys.argv) > 1 and sys.argv[1] or None
    host = None
    filename = None
    if arg:
        if path.exists(arg):
            filename = arg
        else:
            host = arg
    asyncio.run(mission_control(host, filename), debug=True)
