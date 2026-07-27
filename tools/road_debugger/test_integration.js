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
  const packet = Buffer.alloc(68);
  packet.write('RDL1', 0, 'ascii');
  packet.writeUInt8(1, 4);
  packet.writeUInt8(0b101111, 5);
  packet.writeUInt16LE(56, 6);
  packet.writeUInt32LE(7, 8);
  packet.writeUInt32LE(250, 12);
  packet.writeUInt16LE(320, 16);
  packet.writeUInt16LE(240, 18);
  packet.writeUInt8(1, 20);
  packet.writeUInt8(1, 21);
  packet.writeUInt8(1, 22);
  packet.writeInt8(1, 23);
  [60, 120, 20, 12, 58, 96, 24, 24, 160, 140, 80, 90, 75, -725].forEach((value, index) => {
    packet.writeInt16LE(value, 24 + index * 2);
  });
  packet.writeInt16LE(158, 52);
  packet.writeInt16LE(130, 54);
  [[70, 180], [160, 160], [250, 180]].forEach(([x, y], index) => {
    packet.writeInt16LE(x, 56 + index * 4);
    packet.writeInt16LE(y, 58 + index * 4);
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
    await sendUdp(Buffer.from(JSON.stringify({
      packet_type: 'tuning',
      P: 454,
      I: 14,
      gyro: 1,
      dbMode: 1,
      dbUseTangent: 1,
      dbTurnAngle: 25,
      dbReturnBias: 46,
      dbPassDist: 0.14,
      dbHMax: 200,
      dbHTol: 2,
      dbRecoverDps: 55,
      dbBrakePwm: 6000,
      yawHoldRMax: 10,
      udp: 2,
      hwTest: 0,
      hwPwm: 0,
    })));
    await sendUdp(makeRoadPacket());
    await delay(120);

    const statusResponse = await request('GET', '/api/status');
    assert.strictEqual(statusResponse.status, 200);
    assert.strictEqual(statusResponse.json.jsonPackets, 2);
    assert.strictEqual(statusResponse.json.tuningPackets, 1);
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
    assert(pageResponse.text.includes('headingHoldCommand'));
    assert(pageResponse.text.includes('tangentDebugCommand'));
    assert(pageResponse.text.includes('#spd=35;'));
    assert(pageResponse.text.includes('#spd=0;'));
    assert(pageResponse.text.includes('#drive=1;'));
    assert(pageResponse.text.includes('#drive=0;'));
    assert(pageResponse.text.includes('#udp=0;'));
    assert(pageResponse.text.includes('#udp=2;'));
    assert(!pageResponse.text.includes('#spd=20;'));
    assert(!pageResponse.text.includes('#udp=1;'));
    assert(pageResponse.text.includes('remoteUp'));
    assert(pageResponse.text.includes('remoteDown'));
    assert(pageResponse.text.includes('remoteLeft'));
    assert(pageResponse.text.includes('remoteRight'));
    assert(pageResponse.text.includes('左转 120dps'));
    assert(pageResponse.text.includes('右转 120dps'));
    assert(!pageResponse.text.includes('转 60dps'));
    assert(pageResponse.text.includes('tuningControls'));
    assert(pageResponse.text.includes('tuningSnapshotTime'));

    const appResponse = await request('GET', '/app.js');
    assert.strictEqual(appResponse.status, 200);
    const tuningKeys = [
      'P', 'I', 'D', 'spd', 'dirP', 'dirD', 'AIM', 'spd_slow_ratio', 'begin_x',
      'gyro', 'gDbg', 'gTar', 'gOP', 'gOD', 'gIP', 'gII', 'gTMax', 'gRMax',
      'gSign', 'tSign', 'dbMode', 'dbUseTangent', 'dbNormalSpd', 'dbRecSpd',
      'dbTurnAngle', 'dbPassDist',
      'dbReturnBias', 'dbSafeDist', 'dbRpsMps', 'dbViewMax', 'dbViewWait', 'dbHKp',
      'dbHKd', 'dbHMax', 'dbHTol', 'dbRecoverDps', 'dbYawSign', 'dbTurnRps', 'dbForwardRps', 'dbExitRps', 'dbBrakePwm',
      'yawHoldRMax',
      'dbBrakeRelease', 'dbBrakeTimeout', 'dbTestDist', 'circle_exit', 'udp', 'vofa',
      'is_udp_img', 'hwTest', 'hwPwm',
    ];
    tuningKeys.forEach((key) => assert(appResponse.text.includes(`key: '${key}'`), key));
    tuningKeys.forEach((key) => assert(appResponse.text.includes(`\n  ${key}: '`),
      `missing tuning description: ${key}`));
    assert(appResponse.text.includes('RPS是车轮每秒转数'));
    assert(appResponse.text.includes('不是车身角速度'));
    assert(appResponse.text.includes('左轮目标=前进基准RPS+差速RPS'));
    assert(appResponse.text.includes("description.className = 'tuning-description'"));
    assert(!appResponse.text.includes("[2, '六次示教']"));
    assert(!appResponse.text.includes("key: 'dbLearnRps'"));
    assert(!appResponse.text.includes("key: 'dbLearnDist'"));
    assert(!appResponse.text.includes("key: 'dbLearnScale'"));
    assert(appResponse.text.includes("key: 'dbHKp', label: '航向外环P', min: 0, max: 60, step: 0.1, defaultValue: 31"));
    assert(appResponse.text.includes("key: 'dbHMax', label: '绕行最大角速度', min: 0, max: 720, step: 5, unit: 'dps', defaultValue: 505"));
    assert(appResponse.text.includes("const TUNING_MAXES_STORAGE_KEY = 'tuningSliderMaxes'"));
    assert(appResponse.text.includes("maxEditor.className = 'tuning-max-editor'"));
    assert(appResponse.text.includes("maxLabel.textContent = '上限'"));
    assert(appResponse.text.includes("key: 'dbUseTangent', label: '目标处切线参考', kind: 'toggle', defaultValue: 0"));
    assert(appResponse.text.includes("key: 'spd', label: '左右轮前进基准速度', min: 0, max: 60, step: 0.5, unit: 'RPS', defaultValue: 0"));
    assert(appResponse.text.includes("key: 'dbRecSpd', label: '识别阶段前进基准速度', min: 0, max: 40, step: 0.5, unit: 'RPS', defaultValue: 11"));
    assert(appResponse.text.includes("key: 'dbTurnAngle', label: '向外转角', min: 0, max: 90, step: 1, unit: 'deg', defaultValue: 51"));
    assert(appResponse.text.includes("key: 'dbReturnBias', label: '回赛道预偏角', min: 0, max: 91, step: 1, unit: 'deg', defaultValue: 52"));
    assert(appResponse.text.includes("key: 'dbPassDist', label: '最短斜行距离', min: 0, max: 2, step: 0.01, unit: 'm', defaultValue: 0"));
    assert(appResponse.text.includes("key: 'dbTurnRps', label: '转出阶段前进基准速度', min: 0, max: 40, step: 1, unit: 'RPS', defaultValue: 15"));
    assert(appResponse.text.includes("key: 'dbForwardRps', label: '斜行阶段前进基准速度', min: 0, max: 40, step: 1, unit: 'RPS', defaultValue: 20"));
    assert(appResponse.text.includes("key: 'dbExitRps', label: '转入阶段前进基准速度', min: 0, max: 40, step: 1, unit: 'RPS', defaultValue: 15"));
    assert(appResponse.text.includes("key: 'dbViewMax', label: '最大观察夹角', min: 0, max: 90, step: 1, unit: 'deg', defaultValue: 46"));
    assert(appResponse.text.includes("key: 'dbHTol', label: '航向允许误差（退出+1°）', min: 0, max: 10, step: 0.1, unit: 'deg', defaultValue: 4.5"));
    assert(appResponse.text.includes("key: 'dbBrakePwm', label: '主动制动反向PWM', min: 0, max: 7000, step: 50, defaultValue: 6000, hardMax: true"));
    assert(appResponse.text.includes("key: 'hwTest', label: 'PWM1硬件测试', kind: 'toggle', defaultValue: 0"));
    assert(appResponse.text.includes("key: 'hwPwm', label: 'PWM1正向占空比', min: 0, max: 5000, step: 50, defaultValue: 0, hardMax: true"));
    assert(appResponse.text.includes("#remote=0;"));
    assert(appResponse.text.includes("#yawHold="));
    assert(appResponse.text.includes("#tangentDbg="));
    assert(appResponse.text.includes('sendRemoteHeartbeat'));

    const stopResponse = await request('POST', '/api/recording/stop', {});
    assert.strictEqual(stopResponse.status, 200);
    assert.strictEqual(stopResponse.json.recording.eventCount, 3);

    const listResponse = await request('GET', '/api/recordings');
    assert.strictEqual(listResponse.status, 200);
    assert.strictEqual(listResponse.json.recordings.length, 1);
    const fileResponse = await request(
      'GET',
      `/api/recording/file?name=${encodeURIComponent(listResponse.json.recordings[0].name)}`);
    assert.strictEqual(fileResponse.status, 200);
    assert(fileResponse.text.includes('"type":"road"'));
    assert(fileResponse.text.includes('"angleDeg":-7.25'));

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
    assert(recordingText.includes('"type":"tuning"'));
    assert(recordingText.includes('"dbMode":1'));
    assert(recordingText.includes('"dbUseTangent":1'));
    assert(recordingText.includes('"dbHTol":2'));
    assert(recordingText.includes('"yawHoldRMax":10'));
    assert(recordingText.includes('"hwTest":0'));
    assert(recordingText.includes('"hwPwm":0'));
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
