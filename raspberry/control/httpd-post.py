import sys
import BaseHTTPServer
import json
#from jsonobject import fromJson
import control

def handle(path, content):
    print "handle", path, content
    control.write(content) 

class CORSRequestHandler(BaseHTTPServer.BaseHTTPRequestHandler):
    def end_headers (self):
        self.send_header('Access-Control-Allow-Origin', '*')
        BaseHTTPServer.BaseHTTPRequestHandler.end_headers(self)

class MyServer(BaseHTTPServer.BaseHTTPRequestHandler):
    
    # comment away for request logging to stderr
    #def log_message(self, *args): pass

    def do_OPTIONS(self):
        #print self.headers
        self.send_response(204)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
    
    def do_GET(self):
        #print self.headers
        self.send_response(200)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.rfile.write("Control server\n")

    def do_POST(self):
        try:
            self.send_response(204)
            length = self.headers.getheader('content-length')
            if length:
                length = int(length)
                content = self.rfile.read(length)
                #print 'Content:', content
                handle(self.path, content)
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
        except:
            import traceback
            e = traceback.format_exc()
            print >>sys.stderr, e
            self.send_error(500, str(e))


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    #parser.add_argument('-O', '--noObservations', default=False, action='store_true', help="Don't show observations")
    #parser.add_argument('-u', '--url', default='http://mk.tellucloud.com/web', help='Api URL')
    parser.add_argument('--host', default='', help='Bind to host address')
    parser.add_argument('-p', '--port', type=int, default=8000, help='Bind to port')
    parser.add_argument('-v', '--verbose', action='store_true', help="Verbose output")
    #parser.add_argument('--requests', action='store_true', help="Output requests")
    args = parser.parse_args()
    
    httpd = BaseHTTPServer.HTTPServer((args.host, args.port), MyServer)
    print "Serving HTTP on %s:%d ..." % httpd.server_address

    httpd.serve_forever()

