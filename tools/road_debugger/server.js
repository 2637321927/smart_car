'use strict';

const dgram = require('dgram');
const fs = require('fs');
const http = require('http');
const os = require('os');
const path = require('path');
const { URL } = require('url');

const UDP_PORT = Number(process.env.ROAD_DEBUG_UDP_PORT || 8080);
const HTTP_PORT = Number(process.env.ROAD_DEBUG_HTTP_PORT || 8765);
const CAR_IP = process.env.ROAD_DEBUG_CAR_IP || '192.168.43.6';
const CAR_COMMAND_PORT = Number(process.env.ROAD_DEBUG_CAR_PORT || 8082);
const PUBLIC_DIR = path.join(__dirname, 'public');
const RECORDINGS_DIR = process.env.ROAD_DEBUG_RECORDINGS_DIR
  ? path.resolve(process.env.ROAD_DEBUG_RECORDINGS_DIR)
  : path.join(__dirname, 'recordings');

const MIME_TYPES = {
  '.html': 'text/html; charset=utf-8',
  '.js': 'text/javascript; charset=utf-8',
  '.css': 'text/css; charset=utf-8',
  '.json': 'application/json; charset=utf-8',
  '.svg': 'image/svg+xml',
};

const runtime = {
  startedAt: Date.now(),
  latestParams: null,
  latestRoad: null,
  udpPackets: 0,
  jsonPackets: 0,
  roadPackets: 0,
  invalidPackets: 0,
  lastRemote: null,
  lastPacketAt: 0,
  recording: null,
};

const sseClients = new Set();

function localIPv4Addresses() {
  const addresses = [];
  for (const [name, entries] of Object.entries(os.networkInterfaces())) {
    for (const entry of entries || []) {
      if (entry.family === 'IPv4' && !entry.internal) {
        addresses.push({ name, address: entry.address });
      }
    }
  }
  return addresses;
}

function parseRoadPacket(buffer) {
  if (buffer.length < 52 || buffer.toString('ascii', 0, 4) !== 'RDL1') {
    return null;
  }

  const version = buffer.readUInt8(4);
  const flags = buffer.readUInt8(5);
  const headerSize = buffer.readUInt16LE(6);
  const leftCount = buffer.readUInt8(20);
  const centerCount = buffer.readUInt8(21);
  const rightCount = buffer.readUInt8(22);
  const expectedSize = headerSize + (leftCount + centerCount + rightCount) * 4;
  if (version !== 1 || headerSize < 52 || expectedSize > buffer.length) {
    return null;
  }

  let offset = headerSize;
  function readPoints(count) {
    const points = new Array(count);
    for (let index = 0; index < count; index += 1) {
      points[index] = [buffer.readInt16LE(offset), buffer.readInt16LE(offset + 2)];
      offset += 4;
    }
    return points;
  }

  return {
    version,
    sequence: buffer.readUInt32LE(8),
    carTimeMs: buffer.readUInt32LE(12),
    width: buffer.readUInt16LE(16),
    height: buffer.readUInt16LE(18),
    itemFlag: buffer.readInt8(23),
    flags: {
      haveTarget: Boolean(flags & (1 << 0)),
      redValid: Boolean(flags & (1 << 1)),
      plateValid: Boolean(flags & (1 << 2)),
      running: Boolean(flags & (1 << 3)),
      driveBusy: Boolean(flags & (1 << 4)),
    },
    redRect: {
      x: buffer.readInt16LE(24),
      y: buffer.readInt16LE(26),
      w: buffer.readInt16LE(28),
      h: buffer.readInt16LE(30),
    },
    plateRect: {
      x: buffer.readInt16LE(32),
      y: buffer.readInt16LE(34),
      w: buffer.readInt16LE(36),
      h: buffer.readInt16LE(38),
    },
    aim: [buffer.readInt16LE(40), buffer.readInt16LE(42)],
    sourceCounts: {
      left: buffer.readUInt16LE(44),
      center: buffer.readUInt16LE(46),
      right: buffer.readUInt16LE(48),
    },
    lines: {
      left: readPoints(leftCount),
      center: readPoints(centerCount),
      right: readPoints(rightCount),
    },
  };
}

function broadcast(eventName, data) {
  const payload = `event: ${eventName}\ndata: ${JSON.stringify(data)}\n\n`;
  for (const response of sseClients) {
    response.write(payload);
  }
}

function recordingEvent(type, receivedAt, data) {
  if (!runtime.recording) {
    return;
  }

  const event = {
    type,
    t: receivedAt - runtime.recording.startedAt,
    receivedAt,
    data,
  };
  runtime.recording.stream.write(`${JSON.stringify(event)}\n`);
  runtime.recording.eventCount += 1;
}

function sanitizeRecordingName(value) {
  return String(value || 'session')
    .trim()
    .replace(/[^a-zA-Z0-9_-]+/g, '_')
    .replace(/^_+|_+$/g, '')
    .slice(0, 48) || 'session';
}

function timestampForFilename(date) {
  const pad = (value) => String(value).padStart(2, '0');
  return `${date.getFullYear()}${pad(date.getMonth() + 1)}${pad(date.getDate())}_` +
    `${pad(date.getHours())}${pad(date.getMinutes())}${pad(date.getSeconds())}`;
}

function startRecording(name) {
  if (runtime.recording) {
    throw new Error('已经在录制');
  }

  fs.mkdirSync(RECORDINGS_DIR, { recursive: true });
  const startedAt = Date.now();
  const filename = `${timestampForFilename(new Date(startedAt))}_${sanitizeRecordingName(name)}.jsonl`;
  const filePath = path.join(RECORDINGS_DIR, filename);
  const stream = fs.createWriteStream(filePath, { encoding: 'utf8' });
  runtime.recording = { filename, filePath, stream, startedAt, eventCount: 0 };
  stream.write(`${JSON.stringify({
    type: 'meta',
    version: 1,
    startedAt,
    udpPort: UDP_PORT,
    carIp: CAR_IP,
  })}\n`);
  broadcast('recording', recordingState());
  return recordingState();
}

function stopRecording() {
  if (!runtime.recording) {
    return Promise.resolve(null);
  }

  const recording = runtime.recording;
  runtime.recording = null;
  return new Promise((resolve, reject) => {
    recording.stream.end(() => {
      broadcast('recording', recordingState());
      resolve({
        filename: recording.filename,
        eventCount: recording.eventCount,
        durationMs: Date.now() - recording.startedAt,
      });
    });
    recording.stream.once('error', reject);
  });
}

function recordingState() {
  if (!runtime.recording) {
    return { active: false };
  }
  return {
    active: true,
    filename: runtime.recording.filename,
    eventCount: runtime.recording.eventCount,
    durationMs: Date.now() - runtime.recording.startedAt,
  };
}

function listRecordings() {
  fs.mkdirSync(RECORDINGS_DIR, { recursive: true });
  return fs.readdirSync(RECORDINGS_DIR)
    .filter((name) => name.endsWith('.jsonl'))
    .map((name) => {
      const stats = fs.statSync(path.join(RECORDINGS_DIR, name));
      return { name, size: stats.size, modifiedAt: stats.mtimeMs };
    })
    .sort((left, right) => right.modifiedAt - left.modifiedAt);
}

function statusPayload() {
  return {
    startedAt: runtime.startedAt,
    udpPort: UDP_PORT,
    httpPort: HTTP_PORT,
    carIp: CAR_IP,
    carCommandPort: CAR_COMMAND_PORT,
    localAddresses: localIPv4Addresses(),
    udpPackets: runtime.udpPackets,
    jsonPackets: runtime.jsonPackets,
    roadPackets: runtime.roadPackets,
    invalidPackets: runtime.invalidPackets,
    lastPacketAt: runtime.lastPacketAt,
    lastRemote: runtime.lastRemote,
    recording: recordingState(),
  };
}

function sendJson(response, statusCode, value) {
  const body = JSON.stringify(value);
  response.writeHead(statusCode, {
    'Content-Type': 'application/json; charset=utf-8',
    'Content-Length': Buffer.byteLength(body),
    'Cache-Control': 'no-store',
  });
  response.end(body);
}

function readJsonBody(request) {
  return new Promise((resolve, reject) => {
    const chunks = [];
    let size = 0;
    request.on('data', (chunk) => {
      size += chunk.length;
      if (size > 64 * 1024) {
        reject(new Error('请求内容过大'));
        request.destroy();
        return;
      }
      chunks.push(chunk);
    });
    request.on('end', () => {
      try {
        const text = Buffer.concat(chunks).toString('utf8');
        resolve(text ? JSON.parse(text) : {});
      } catch (error) {
        reject(new Error('JSON格式错误'));
      }
    });
    request.on('error', reject);
  });
}

function serveStatic(requestPath, response) {
  const relative = requestPath === '/' ? 'index.html' : requestPath.replace(/^\/+/, '');
  const publicRoot = path.resolve(PUBLIC_DIR);
  const filePath = path.resolve(PUBLIC_DIR, relative);
  if ((filePath !== publicRoot && !filePath.startsWith(`${publicRoot}${path.sep}`)) ||
      !fs.existsSync(filePath)) {
    sendJson(response, 404, { error: '文件不存在' });
    return;
  }

  const stats = fs.statSync(filePath);
  if (!stats.isFile()) {
    sendJson(response, 404, { error: '文件不存在' });
    return;
  }

  response.writeHead(200, {
    'Content-Type': MIME_TYPES[path.extname(filePath)] || 'application/octet-stream',
    'Content-Length': stats.size,
    'Cache-Control': 'no-store',
  });
  fs.createReadStream(filePath).pipe(response);
}

function start() {
  fs.mkdirSync(RECORDINGS_DIR, { recursive: true });
  const udpReceiver = dgram.createSocket('udp4');
  const commandSocket = dgram.createSocket('udp4');

  udpReceiver.on('message', (message, remote) => {
    const receivedAt = Date.now();
    runtime.udpPackets += 1;
    runtime.lastPacketAt = receivedAt;
    runtime.lastRemote = `${remote.address}:${remote.port}`;

    if (message[0] === 0x7b) {
      try {
        const params = JSON.parse(message.toString('utf8'));
        runtime.latestParams = params;
        runtime.jsonPackets += 1;
        const event = { receivedAt, params };
        broadcast('params', event);
        recordingEvent('params', receivedAt, params);
      } catch (error) {
        runtime.invalidPackets += 1;
      }
      return;
    }

    const road = parseRoadPacket(message);
    if (road) {
      runtime.latestRoad = road;
      runtime.roadPackets += 1;
      const event = { receivedAt, road };
      broadcast('road', event);
      recordingEvent('road', receivedAt, road);
      return;
    }

    runtime.invalidPackets += 1;
  });

  udpReceiver.on('error', (error) => {
    console.error(`[UDP] ${error.message}`);
  });

  udpReceiver.bind(UDP_PORT, '0.0.0.0', () => {
    console.log(`[UDP] 正在监听 0.0.0.0:${UDP_PORT}`);
  });

  const httpServer = http.createServer(async (request, response) => {
    const requestUrl = new URL(request.url, `http://${request.headers.host || '127.0.0.1'}`);

    try {
      if (request.method === 'GET' && requestUrl.pathname === '/events') {
        response.writeHead(200, {
          'Content-Type': 'text/event-stream; charset=utf-8',
          'Cache-Control': 'no-cache',
          Connection: 'keep-alive',
        });
        response.write('retry: 1000\n\n');
        sseClients.add(response);
        response.write(`event: status\ndata: ${JSON.stringify(statusPayload())}\n\n`);
        if (runtime.latestParams) {
          response.write(`event: params\ndata: ${JSON.stringify({
            receivedAt: runtime.lastPacketAt,
            params: runtime.latestParams,
          })}\n\n`);
        }
        if (runtime.latestRoad) {
          response.write(`event: road\ndata: ${JSON.stringify({
            receivedAt: runtime.lastPacketAt,
            road: runtime.latestRoad,
          })}\n\n`);
        }
        request.on('close', () => sseClients.delete(response));
        return;
      }

      if (request.method === 'GET' && requestUrl.pathname === '/api/status') {
        sendJson(response, 200, statusPayload());
        return;
      }

      if (request.method === 'GET' && requestUrl.pathname === '/api/recordings') {
        sendJson(response, 200, { recordings: listRecordings() });
        return;
      }

      if (request.method === 'GET' && requestUrl.pathname === '/api/recording/file') {
        const filename = path.basename(requestUrl.searchParams.get('name') || '');
        const filePath = path.join(RECORDINGS_DIR, filename);
        if (!filename.endsWith('.jsonl') || !fs.existsSync(filePath)) {
          sendJson(response, 404, { error: '录像不存在' });
          return;
        }
        const stats = fs.statSync(filePath);
        response.writeHead(200, {
          'Content-Type': 'application/x-ndjson; charset=utf-8',
          'Content-Length': stats.size,
          'Cache-Control': 'no-store',
        });
        fs.createReadStream(filePath).pipe(response);
        return;
      }

      if (request.method === 'POST' && requestUrl.pathname === '/api/recording/start') {
        const body = await readJsonBody(request);
        sendJson(response, 200, startRecording(body.name));
        return;
      }

      if (request.method === 'POST' && requestUrl.pathname === '/api/recording/stop') {
        sendJson(response, 200, { recording: await stopRecording() });
        return;
      }

      if (request.method === 'POST' && requestUrl.pathname === '/api/command') {
        const body = await readJsonBody(request);
        const command = String(body.command || '').trim();
        const ip = String(body.ip || CAR_IP).trim();
        const port = Number(body.port || CAR_COMMAND_PORT);
        if (!command || !ip || !Number.isInteger(port) || port < 1 || port > 65535) {
          sendJson(response, 400, { error: '指令、IP或端口无效' });
          return;
        }
        const payload = Buffer.from(command, 'utf8');
        await new Promise((resolve, reject) => {
          commandSocket.send(payload, port, ip, (error) => error ? reject(error) : resolve());
        });
        sendJson(response, 200, { sent: true, command, ip, port });
        return;
      }

      if (request.method === 'GET') {
        serveStatic(requestUrl.pathname, response);
        return;
      }

      sendJson(response, 404, { error: '接口不存在' });
    } catch (error) {
      sendJson(response, 400, { error: error.message });
    }
  });

  httpServer.listen(HTTP_PORT, '127.0.0.1', () => {
    console.log(`[WEB] http://127.0.0.1:${HTTP_PORT}`);
    const addresses = localIPv4Addresses();
    if (addresses.length) {
      console.log('[提示] 小车TARGET_IP应设置为以下地址之一：');
      addresses.forEach((entry) => console.log(`  ${entry.name}: ${entry.address}`));
    }
  });

  const heartbeat = setInterval(() => {
    broadcast('status', statusPayload());
  }, 1000);

  async function shutdown() {
    clearInterval(heartbeat);
    await stopRecording().catch(() => null);
    udpReceiver.close();
    commandSocket.close();
    httpServer.close(() => process.exit(0));
  }

  process.on('SIGINT', shutdown);
  process.on('SIGTERM', shutdown);

  return { udpReceiver, httpServer, commandSocket };
}

if (require.main === module) {
  start();
}

module.exports = { parseRoadPacket, start };
