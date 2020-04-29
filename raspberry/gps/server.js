const Net = require('net');
const port = 8080;

const server = new Net.Server();
server.listen(port, function() {
    console.log(`Server on socket localhost:${port}`);
});

connections = new Set()

server.on('connection', function(socket) {
    console.log('Connection open');
    connections.add(socket);

    // The server can also receive data from the client by reading from its socket.
    socket.on('data', function(chunk) {
        // console.log(`Data received from client: ${chunk.toString()}`);
    });

    socket.on('end', function() {
        console.log('Closing connection');
        connections.delete(socket);
    });

    // Don't forget to catch error, for your own sake.
    socket.on('error', function(err) {
        console.log(`Error: ${err}`);
    });
});


function broadcast(msg) {
    connections.forEach(socket => socket.write(msg));
};
