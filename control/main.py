import websockets
import asyncio
# import aiostream
import logging
import time
from datetime import datetime
import GPS
import RobotState
import RobotMessages
# import control
from state import State
from shapely.geometry import JOIN_STYLE

logger = logging.getLogger(__name__)


async def controld_reader(websocket, incoming):
    try:
        while True:
            msg = await websocket.recv()
            t = datetime.utcnow()
            msg = msg.strip()
            logger.debug("ROBOT %s", msg)
            await incoming.put((t, RobotMessages.process(msg)))
            # incoming.put_nowait((t, RobotMessages.process(msg)))
    except:
        logger.exception("controld reader")


async def controld_writer(websocket, outgoing):
    try:
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
    except:
        logger.exception("controld writer")


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
                logger.info("Robot read/write failed, restarting...")
                for task in pending:
                    task.cancel()                
        except:
            logger.exception("Controld connection failed")
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
                    yield datetime.utcnow(), RobotMessages.process(msg)
        except:
            logger.exception("From controld failed")
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


def simulated_control(control, plot):
    from RobotModel import RobotModel
    # FIXME: configurable initial state
    model = RobotModel(State(-10, -2, 0, 0))
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


async def shutdown(loop):
    tasks = [t for t in asyncio.all_tasks() if t is not asyncio.current_task()]
    for task in tasks:
        task.cancel() 
    logging.info(f"Cancelling {len(tasks)} outstanding tasks")
    await asyncio.gather(*tasks, return_exceptions=True)
    loop.stop()


def handle_exception(loop, context):
    # context["message"] will always be there; but context["exception"] may not
    msg = context.get("exception", context["message"])
    logging.error(f"Caught exception: {msg}")
    logging.info("Shutting down...")
    asyncio.create_task(shutdown(loop))


async def track(stream, yaw=0, speed=0):
    import state
    import tracking

    p = RobotState.RobotState()
    tracker = tracking.ExtendedKalmanFilterTracker()

    # For every message from GPS/robot or on timeout
    # run control loop and provide output to robot - OR - on fail safe, stop robot
    # (possibly with the option to resume the control loop, saving the last known good position)

    async for t, o in stream:
        # logger.debug("track input {%s} {%s}", t, o)
        if t is None or o is None:
            continue

        # returns (t, dt, z, u) or None
        m = p.update(t, o)
        # logger.debug("track input %s %r -> %r", t, o, m)

        if not p.battery_ok:
            logger.error("Battery low: %s", p.battery)
            return

        if not p.power_ok:
            logger.error("Power high: %s", p.power)
            return

        if m is not None:
            # logger.debug("track input %s %r -> %r", t, o, m)
            # feed tracking.track()
            _, dt, z, ud, hdop = m
            # print("TRACK STREAM", dt, z, ud)
            s = tracker.update2(z, ud, hdop, dt)
            s = state.from_array(s)
            logger.debug("STATE %g %g %g %g", s.x, s.y, s.theta, s.speed)
            yield s


"""
ROS like topics for event processing!?
topic.publish()
topic.subscribe()


control():
  gps -> tracker.update() at 2Hz or whatever...
  move -> tracker.set()
  both via robot state
  
  tracker update -> control.update()
  fail safe -> control.stop/save (pause)
  gps timeout -> control.stop/save/pause (connection delay)  
"""


async def realtime_control(host, control, plot, cut):    
    incoming = asyncio.Queue()
    outgoing = asyncio.Queue()
    # Keep references to tasks to avoid them being stopped instantly
    tc = asyncio.create_task(controld_connection(host, incoming, outgoing))
    tg = asyncio.create_task(gps_connection(host, incoming))

    # incoming -> track -> control -> outgoing
    async for state in track(incoming_generator(incoming), 0, 0):
        t = time.time()
        plot.update(state)
        if control.end(t, state):
            break
        speed, omega = control.update(t, state)
        # logger.debug("Update %s -> %r %r", state, speed, omega)
        if speed is not None and omega is not None:
            t = datetime.utcnow()
            m = RobotMessages.CutCommand(cut)
            await outgoing.put((t, m))

            # FIXME: figure out what the last Time is from the robot, keep it in state?
            t = datetime.utcnow()
            m = RobotMessages.MoveCommand(speed, omega, timeout)
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


def replay2(file, plot):
    i = 0
    for line in file:
        print("REPLAY", line)
        print("REPLAY", RobotState.process_line(line))


def mission_control(host, filename):
    from Map import Config
    from Plotting import Plot
    from control import CompositeControl2, ScanHLine, FenceBumps, RingControls, FenceShrink, PathControls

    config = Config("mission.yaml")
    if config.aoi:
        aoi = config.fence.intersection(config.aoi)
    else:
        aoi = config.fence
    speed = config.mission.speed or 0.05
    omega = config.mission.omega or 0.2
    cut = config.mission.cut or 20

    # controls = RingControls(fence.exterior.coords, speed, omega)
    aoi = aoi.buffer(-0.1, join_style=JOIN_STYLE.mitre)
    # controls = FenceBumps(aoi, speed, omega)
    controls = FenceShrink(config.fence, aoi, speed, omega)
    # controls = ScanHLine(-10, -4.5, 13, -1.5, speed, omega) # midten - lang
    # controls = ScanHLine(-10.5, -1.6, 17, -0.9, speed, omega) # roser
    # controls = ScanHLine(-10, -7.5, -1, -4, speed, omega) # slackline - gml gran
    # controls = ScanHLine(-10, -9, -6, -4, speed, omega) # slackline - paere
    
    # controls = ScanHLine(-8, -5, 3, -1.2, speed, omega) # midten med mest gras++
    # controls = ScanHLine(-8, -3, 3, -1.2, speed, omega) # midten med mest gras
    # controls = ScanHLine(-11, -6, -5, -0.4, speed, omega) # mot epler
    # controls = ScanHLine(-2, -4, 2, -1, speed, omega) # test
    # from control import LineControlTest
    # controls = LineControlTest()
    # controls = PathControls(aoi.exterior.coords, speed, omega)

    control = CompositeControl2(controls)
    logger.info("Starting mission control for %s", controls)

    plot = Plot(frames_per_plot=host is None and 150 or 1)
    plot.add_shape(config.fence, facecolor="khaki")
    plot.add_shape(aoi, facecolor="darkkhaki")
    plot.pause()
    # input("Wait for key press")

    if host is None:
        if filename:
            replay(open(filename), plot)
        else:
            simulated_control(control, plot)
    else:
        asyncio.run(realtime_control(host, control, plot, cut), debug=True)
        asyncio.run(shutdown(asyncio.get_event_loop()))

    plot.pause()
    plot.show()


if __name__ == "__main__":
    import sys, yaml
    from os import path
    import logging.config
    with open("logging.yaml") as f:
        logging.config.dictConfig(yaml.full_load(f))
    loop = asyncio.get_event_loop()
    loop.set_exception_handler(handle_exception)

    arg = len(sys.argv) > 1 and sys.argv[1] or None
    host = None
    filename = None
    if arg:
        if path.exists(arg):
            filename = arg
        else:
            host = arg
    mission_control(host, filename)
