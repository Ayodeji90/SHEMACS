const express = require('express');
const http = require('http');
const socketIo = require('socket.io');
const mqtt = require('mqtt');

const app = express();
const server = http.createServer(app);
const io = socketIo(server);

// MQTT setup
const mqttClient = mqtt.connect('mqtt://your_mqtt_broker_ip'); // e.g., mqtt://192.168.1.100
let devices = {}; // Store device data
 
mqttClient.on('connect', () => {
  console.log('Connected to MQTT broker');
  mqttClient.subscribe('home/power+#');
  mqttClient.subscribe('home/relay+#');
  mqttClient.subscribe('home/temp');
  mqttClient.subscribe('home/light');
});

mqttClient.on('message', (topic, message) => {
  const msg = message.toString();
  if (topic.startsWith('home/power')) {
    const deviceId = topic.split('/')[2];
    if (!devices[deviceId]) devices[deviceId] = { power: 0, relay: 'OFF' };
    devices[deviceId].power = parseFloat(msg);
  } else if (topic.startsWith('home/relay') && topic.endsWith('/state')) {
    const deviceId = topic.split('/')[2];
    if (!devices[deviceId]) devices[deviceId] = { power: 0, relay: 'OFF' };
    devices[deviceId].relay = msg;
  } else if (topic === 'home/temp') {
    devices.temp = parseFloat(msg);
  } else if (topic === 'home/light') {
    devices.light = parseFloat(msg);
  }
  io.emit('update', devices); // Send updated data to clients
});

// Serve static files
app.use(express.static('public'));

// Start server
server.listen(3000, () => {
  console.log('Server running on http://localhost:3000');
});

// Socket.IO for real-time updates
io.on('connection', (socket) => {
  socket.emit('update', devices); // Send current state to new clients
  socket.on('toggle', (deviceId) => {
    const newState = devices[deviceId].relay === 'ON' ? 'OFF' : 'ON';
    mqttClient.publish(`home/relay${deviceId}`, newState);
  });
  socket.on('addDevice', (deviceId) => {
    if (!devices[deviceId]) {
      devices[deviceId] = { power: 0, relay: 'OFF' };
      io.emit('update', devices);
    }
  });
});