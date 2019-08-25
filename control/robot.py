import websocket
import time
import threading
import logging
import random

logger = logging.getLogger("robot")

#url = "ws://localhost:8000/control"
#url = "ws://gardenbot.local:8000/control"


def Speed(speed):
    return "t %d" % speed

def Turn(speed):
    return "r %d" % speed

Stop = "."
Reset = "!"
Heartbeat = "heartbeat"



_lock = threading.Lock()
_ws = None
battery_level = None

def on_message(ws, message):
    message = message.strip()
    cols = message.split()
    cmd = cols[0]
    if cmd == "Control":
        logger.info("From robot: %s", message)
    elif cmd == "Battery":
        logger.info("From robot: %s", message)
        global battery_level
        battery_level = float(cols[1])
    else:
        logger.debug("From robot: %s", message)

def on_error(ws, error):
    logger.warning(error)

def on_close(ws):
    global _ws
    with _lock:
        _ws = None
    logger.warning("Robot connection closed")

def on_open(ws):
    global _ws
    with _lock:
        _ws = ws
    logger.info("Robot connection open")


def start(url):
    # websocket.enableTrace(True)
    def run():
        while True:
            ws = websocket.WebSocketApp(url,
                                      on_message = on_message,
                                      on_error = on_error,
                                      on_open = on_open,
                                      on_close = on_close)
            ws.run_forever()
            time.sleep(0.3)

    t = threading.Thread(target=run)
    t.setDaemon(True)
    t.start()


def send(msg):
    msg = msg.strip()
    with _lock:
        logging.info("To robot: %s", msg)
        #return
        if _ws is not None:
            _ws.send(msg + "\n")


def schedule(plan):
    complete = threading.Event()
    def run():
        for cmd, dt in plan:
            logger.debug("Schedule %s %.1f", cmd, dt)
            send(cmd)
            if dt > 0:
                time.sleep(dt)
        complete.set()
    t = threading.Thread(target=run)
    t.setDaemon(True)
    t.start()
    return complete


def avoid(speed, turn):
    # would help to have heading information of the robot, could then do a more
    # intelligent manoeuvre.
    logger.info("Starting avoidance")
    schedule([
        # reverse
        (Speed(-speed), 5), (Speed(-speed), 3 * random.random()), (Speed(0), 0),
        # turn
        (Turn(turn), 4 + 2 * random.random()), (Turn(0), 0),
        # go forward, not in main control
        (Speed(speed), 0),
        ])


if __name__ == '__main__':
    logging.basicConfig()
    start("ws://localhost:8000/control")
    while True:
        time.sleep(1)
        try:
            print("HB")
            send(Heartbeat)
        except:
            pass
