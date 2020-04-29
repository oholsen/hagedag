const { NtripClient } = require('ntrip-client');
const projector = require('ecef-projector');
const configYaml = require('config-yaml');
 
const config = configYaml(`config.yaml`);
const options = config.ntrip;
const position = config.position;
options.xyz = projector.project(position.lat, position.lon, position.alt);

const client = new NtripClient(options);

client.on('data', (data) => {
  console.log(data);
});

client.on('close', () => {
  console.log('client close');
});

client.on('error', (err) => {
  console.log(err);
});

client.run();
