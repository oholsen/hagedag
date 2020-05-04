const { NtripClient } = require('ntrip-client');
const projector = require('ecef-projector');
const SerialPort = require('serialport');
const Net = require('net');
const configYaml = require('config-yaml');
 
const config = configYaml(`config.yaml`);
const options = config.ntrip;
const position = config.position;
options.xyz = projector.project(position.lat, position.lon, position.alt);

const tcpPort = config.server.port;
const serialPort = new SerialPort(config.gps.device, config.gps.settings);


// Switches the port into "flowing mode"
serialPort.on('data', function (data) {
  // console.log('Data:', data.toString())
  data = data.toString();
  broadcast(data);
  // lines = data.split(/\r?\n/)
  // lines.forEach(line => console.log("Line", line))
  // lines.forEach(line => line.startsWith("$GNGGA") && console.log("Line", line))
})


const client = new NtripClient(options);

client.on('data', (data) => {
  // console.log(data);
  serialPort.write(data);
});

client.on('close', () => {
  console.log('NtripClient closed');
});

client.on('error', (err) => {
  console.log("NtripClient error:", err);
});

client.run();



const server = new Net.Server();
const connections = new Set()

server.listen(tcpPort, function() {
  console.log(`Server on socket ${tcpPort}`);
});


server.on('connection', function(socket) {
  console.log('Connection open');
  connections.add(socket);

  socket.on('data', function(data) {
    // console.log(`Data from client: ${data.toString()}`);
  });

  socket.on('end', function() {
    console.log('Closing connection');
    connections.delete(socket);
  });

  socket.on('error', function(err) {
    console.log("Connection error:", err);
    connections.delete(socket);
  });
});


function broadcast(msg) {
  connections.forEach(socket => socket.write(msg));
};
