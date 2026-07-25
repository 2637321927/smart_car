'use strict';

const assert = require('assert');
const dgram = require('dgram');
const fs = require('fs');
const http = require('http');
const os = require('os');
const path = require('path');

const testRoot = fs.mkdtempSync(path.join(os.tmpdir(), 'road-debugger-'));
process.env.ROAD_DEBUG_UDP_PORT = '18080';
process.env.ROAD_DEBUG_HTTP_PORT = '18765';
process.env.ROAD_DEBUG_RECORDINGS_DIR = testRoot;

const { start } = require('./server');

function request(method, requestPath, body) {
  return new Promise((resolve, reject) => {
    const encoded = body === undefined ? null : Buffer.from(JSON.stringify(body));
    const req = http.request({
      hostname: '127.0.0.1',
      port: 18765,
      method,
      path: requestPath,
      headers: encoded ? {
        'Content-Type': 'application/json',
        'Content-Length': encoded.length,
      } : {},
    }, (response) => {
      const chunks = [];
      response.on('data', (chunk) => chunks.push(chunk));
      response.on('end', () => {
        const text = Buffer.concat(chunks).toString('utf8');
        const isJson = String(response.headers['content-type'] || '').includes('application/json');
        resolve({
          status: response.statusCode,
          text,
          json: isJson && text ? JSON.parse(text) : null,
        });
      });
    });
    req.on('error', reject);
    if (encoded) req.write(encoded);
    req.end();
  });
}

function makeRoadPacket() {
  const packet = Buffer.alloc(64);
  packet.write('RDL1', 0, 'ascii');
  packet.writeUInt8(1, 4);
  packet.writeUInt8(0b01111, 5);
  packet.writeUInt16LE(52, 6);
  packet.writeUInt32LE(7, 8);
  packet.writeUInt32LE(250, 12);
  packet.writeUInt16LE(320, 16);
  packet.writeUInt16LE(240, 18);
  packet.writeUInt8(1, 20);
  packet.writeUInt8(1, 21);
  packet.writeUInt8(1, 22);
  packet.writeInt8(1, 23);
  [60, 120, 20, 12, 58, 96, 24, 24, 160, 140, 80, 90, 75, 0].forEach((value, index) => {
    packet.writeInt16LE(value, 24 + index * 2);
  });
  [[70, 180], [160, 160], [250, 180]].forEach(([x, y], index) => {
    packet.writeInt16LE(x, 52 + index * 4);
    packet.writeInt16LE(y, 54 + index * 4);
  });
  return packet;
}

function sendUdp(payload) {
  return new Promise((resolve, reject) => {
    const socket = dgram.createSocket('udp4');
    socket.send(payload, 18080, '127.0.0.1', (error) => {
      socket.close();
      if (error) reject(error);
      else resolve();
    });
  });
}

function delay(milliseconds) {
  return new Promise((resolve) => setTimeout(resolve, milliseconds));
}

async function main() {
  const services = start();
  try {
    await delay(150);
    const startResponse = await request('POST', '/api/recording/start', { name: 'integration' });
    assert.strictEqual(startResponse.status, 200);

    await sendUdp(Buffer.from(JSON.stringify({
      seq: 9,
      run: 1,
      encoder1_speed_avg: 20.1,
      encoder2_speed_avg: 19.9,
      ex_rps1: 20,
      ex_rps2: 20,
      latest_error: 2.5,
      have_target: 1,
      item_flag: 1,
    })));
    await sendUdp(makeRoadPacket());
    await delay(120);

    const statusResponse = await request('GET', '/api/status');
    assert.strictEqual(statusResponse.status, 200);
    assert.strictEqual(statusResponse.json.jsonPackets, 1);
    assert.strictEqual(statusResponse.json.roadPackets, 1);
    assert.strictEqual(statusResponse.json.invalidPackets, 0);

    const pageResponse = await request('GET', '/');
    assert.strictEqual(pageResponse.status, 200);
    assert(pageResponse.text.includes('roadCanvas'));
    assert(pageResponse.text.includes('scopeChannelSelect'));
    assert(pageResponse.text.includes('roadColumnReset'));
    assert(pageResponse.text.includes('driveByLeft'));
    assert(pageResponse.text.includes('driveByRight'));
    assert(pageResponse.text.includes('driveByTestButton'));

    const stopResponse = await request('POST', '/api/recording/stop', {});
    assert.strictEqual(stopResponse.status, 200);
    assert.strictEqual(stopResponse.json.recording.eventCount, 2);

    const listResponse = await request('GET', '/api/recordings');
    assert.strictEqual(listResponse.status, 200);
    assert.strictEqual(listResponse.json.recordings.length, 1);
    const fileResponse = await request(
      'GET',
      `/api/recording/file?name=${encodeURIComponent(listResponse.json.recordings[0].name)}`);
    assert.strictEqual(fileResponse.status, 200);
    assert(fileResponse.text.includes('"type":"road"'));

    const commandResponse = await request('POST', '/api/command', {
      ip: '127.0.0.1',
      port: 19000,
      command: '#run=0;',
    });
    assert.strictEqual(commandResponse.status, 200);
    assert.strictEqual(commandResponse.json.sent, true);

    const recordingText = fs.readFileSync(
      path.join(testRoot, listResponse.json.recordings[0].name),
      'utf8');
    assert(recordingText.includes('"type":"params"'));
    assert(recordingText.includes('"type":"road"'));
    console.log('Road debugger integration test passed');
  } finally {
    services.udpReceiver.close();
    services.commandSocket.close();
    await new Promise((resolve) => services.httpServer.close(resolve));
    fs.rmSync(testRoot, { recursive: true, force: true });
  }
}

main().then(() => process.exit(0)).catch((error) => {
  console.error(error);
  process.exit(1);
});
