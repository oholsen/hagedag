import websocket
import time
import threading
import logging

logger = logging.getLogger()


#url = "ws://localhost:8000/control"
#url = "ws://gardenbot.local:8000/control"

_lock = threading.Lock()
_ws = None

def on_message(ws, message):
    logger.debug("From robot: %s", message.strip())

def on_error(ws, error):
    logger.info(error)

def on_close(ws):
    global _ws
    with _lock:
        _ws = None
    logger.warn("### closed ###")

def on_open(ws):
    global _ws
    with _lock:
        _ws = ws
    logger.info("### open ###")


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

def Speed(speed):
    return "t %d" % speed

def Turn(speed):
    return "r %d" % speed

def schedule(plan):
    def run():
        for cmd, dt in plan:
            send(cmd)
            if dt > 0:
                time.sleep(dt)
    t = threading.Thread(target=run)
    t.setDaemon(True)
    t.start()
    logger.debug("Starting avoidance")

def avoid(speed, turn):
    schedule([
        (Speed(-speed), 4), (Speed(-speed), 4), (Speed(0), 0), 
        (Turn(turn), 8), (Turn(0), 0),
        (Speed(speed), 0),
        ])

if __name__ == '__main__':
    logging.basicConfig()
    start("ws://localhost:8000/control")
    while True:
        time.sleep(1)
        try:
            print("HB")
            send("heartbeat")
        except:
            pass
