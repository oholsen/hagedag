const SerialPort = require('serialport')
const port = new SerialPort('/dev/ttyACM0', {
  baudRate: 38400
})

// Switches the port into "flowing mode"
port.on('data', function (data) {
  // console.log('Data:', data.toString())
  data = data.toString()
  lines = data.split(/\r?\n/)
  // lines.forEach(line => console.log("Line", line))
  lines.forEach(line => line.startsWith("$GNGGA") && console.log("Line", line))
})



