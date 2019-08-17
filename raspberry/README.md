# README

# Architecture


Python code: A bridge between HTTP and serial communication.
WebSocket input goes to serial port, streaming output of serial port to all WS connections.


Static content (and .js) is served by apache.



Running on PC...

// with auto refresh:
$ npm install -g light-server 
$ light-server -s . -w gui/**


// No auto refresh:
$ npm install connect serve-static
$ node server.js