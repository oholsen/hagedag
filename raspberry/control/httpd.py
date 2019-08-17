#main program and imports for standalone purpose       
import sys
import threading
from SocketServer import ThreadingMixIn
from BaseHTTPServer import HTTPServer
from BaseHTTPServer import BaseHTTPRequestHandler
from SimpleHTTPServer import SimpleHTTPRequestHandler
from HTTPWebSocketsHandler import HTTPWebSocketsHandler
from base64 import b64encode


'''
A bridge between HTTP and serial communication.
'''

# TODO: add control reader sending to all connections
import control

connections = set()
lock = threading.Lock()



# TODO: include???
# OR use as base for ws handler?
class CORSRequestHandler(BaseHTTPRequestHandler):
    def end_headers (self):
        self.send_header('Access-Control-Allow-Origin', '*')
        BaseHTTPRequestHandler.end_headers(self)

def broadcast(message):
    with lock:
        for c in connections:
            c.send_message(message)

def gen():
    import time
    i = 0
    while True:
        message = "hello %d\n" % i
        broadcast(message)
        time.sleep(1)
        i += 1


def readControl():
    while True:
        line = control.stm.readline()
        #print "FROM CONTROL %r" % line
        broadcast(line)


class WSSimpleEcho(HTTPWebSocketsHandler):
    def on_ws_message(self, message):
        if message is None:
            message = ''
        message = str(message)
        self.log_message('websocket received "%s"', message)
        control.handle(message)
        # echo message back to client
        # self.send_message(message)

    def on_ws_connected(self):
        self.log_message('%s','websocket connected')
        with lock:
            connections.add(self)

    def on_ws_closed(self):
        self.log_message('%s','websocket closed')
        with lock:
            connections.remove(self)


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    """Handle requests in a separate thread."""


def _ws_main():    
    try:
        # Replace WSSimpleEcho with your own subclass of HTTPWebSocketHandler
        server = ThreadedHTTPServer((args.host, args.port), WSSimpleEcho)
        server.daemon_threads = True
        server.protocol_version = "HTTP/1.1"
        print(server)
        print(server.protocol_version)
        server.auth = b64encode(credentials)
        if secure:
            import ssl
            # server.auth = b64encode(credentials)
            server.socket = ssl.wrap_socket(server.socket, certfile='./server.pem', server_side=True)
            print('started https server at port %d' % args.port)
        else: 
            print('started http server at port %d' % args.port)
        server.serve_forever()
    except KeyboardInterrupt:
        print('^C received, shutting down server')
        server.socket.close()


if __name__ == '__main__':
    if 0:
        if len(sys.argv) > 1:
            port = int(sys.argv[1])
        else:
            port = 8000
        if len(sys.argv) > 2:
            secure = str(sys.argv[2]).lower() == "secure"
        else:
            secure = False
        if len(sys.argv) > 3:
            credentials = str(sys.argv[3])
        else:
            credentials = ""

    secure = False
    credentials = ""

    import argparse
    parser = argparse.ArgumentParser()
    #parser.add_argument('-O', '--noObservations', default=False, action='store_true', help="Don't show observations")
    #parser.add_argument('-u', '--url', default='http://mk.tellucloud.com/web', help='Api URL')
    parser.add_argument('--host', default='', help='Bind to host address')
    parser.add_argument('-p', '--port', type=int, default=8000, help='Bind to port')
    parser.add_argument('-v', '--verbose', action='store_true', help="Verbose output")
    #parser.add_argument('--requests', action='store_true', help="Output requests")
    args = parser.parse_args()


    #thread = threading.Thread(target=gen)
    thread = threading.Thread(target=readControl)
    thread.daemon = True
    thread.start()

    _ws_main()
