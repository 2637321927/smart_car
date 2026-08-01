'use strict';

const dgram = require('dgram');
const fs = require('fs');
const http = require('http');
const os = require('os');
const path = require('path');
const { URL } = require('url');

const UDP_PORT = Number(process.env.ROAD_DEBUG_UDP_PORT || 8080);
const HTTP_PORT = Number(process.env.ROAD_DEBUG_HTTP_PORT || 8765);
const CAR_IP = process.env.ROAD_DEBUG_CAR_IP || '192.168.43.93';
const CAR_COMMAND_PORT = Number(process.env.ROAD_DEBUG_CAR_PORT || 8082);
const TARGET_RESULT_DEDUPE_MS = 2000;
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
  latestTuning: null,
  latestRoad: null,
  latestTargetResult: null,
  lastTargetResultKey: '',
  lastTargetResultAt: 0,
  udpPackets: 0,
  jsonPackets: 0,
  tuningPackets: 0,
  controlPackets: 0,
  roadPackets: 0,
  invalidPackets: 0,
  lastRemote: null,
  lastPacketAt: 0,
  lastControlAt: 0,
  lastTuningAt: 0,
  lastRoadAt: 0,
  recording: null,
};

const sseClients = new Set();

const CONTROL_STATE_NAMES = [
  'IDLE', 'APPROACH', 'WAIT_VIEW', 'INFER', 'WAIT_BRAKE',
  'START_MOTION_PENDING', 'TURN_OUT', 'PASS_SHORT', 'TURN_TO_TRACK',
  'FOLLOW_SIDE_LINE', 'RECOVER_CENTER_LINE', 'FINISH_PENDING',
];
const CONTROL_ABORT_NAMES = [
  'none', 'view_timeout', 'target_geometry_invalid', 'gyro_not_ready',
  'gyro_stale', 'phase_timeout', 'brake_timeout',
];
const CONTROL_INT_FIELDS = [
  'gyro_timeout', 'to_id', 'to_total', 'selected_speed', 'drive_state_code',
  'drive_abort_reason_code', 'drive_brake_pwm', 'drive_brake_elapsed_ms',
  'drive_infer_valid_count', 'red_candidate_count', 'red_contour_area',
  'drive_detection_stage', 'circle_type', 'cross_type', 'track_type', 'item_flag',
];
const CONTROL_FLOAT_FIELDS = [
  'encoder1_speed_avg', 'encoder2_speed_avg', 'latest_error', 'ex_rps1', 'ex_rps2',
  'current_pwm1', 'current_pwm2', 'P1_motor', 'P2_motor', 'D1_motor', 'D2_motor',
  'gyro_target_dps', 'gyro_dps', 'y_guard_aim_dy_px', 'y_guard_target_dps',
  'to_used', 'to_target', 'track_tangent_deg', 'drive_yaw_deg',
  'drive_target_yaw_deg', 'drive_heading_error_deg', 'drive_track_heading_deg',
  'drive_target_track_heading_deg', 'drive_view_angle_deg', 'drive_target_distance_m',
  'drive_distance_since_trigger_m', 'drive_phase_distance_m',
  'drive_target_yaw_rate_dps', 'drive_turn_rps', 'circle_exit_m', 'odom_m', 'AIM',
  'drive_test_target_distance_m',
];

function parseTargetResultPacket(value) {
  const eventId = Number(value?.event_id);
  const result = Number(value?.result);
  const text = typeof value?.text === 'string' ? value.text.trim() : '';
  if (value?.packet_type !== 'target_result' ||
      !Number.isSafeInteger(eventId) || eventId < 0 || eventId > 0xffffffff ||
      !Number.isInteger(result) || result < -1 || result > 2 ||
      !text || Buffer.byteLength(text, 'utf8') >= 128) {
    return null;
  }
  return { eventId, result, text };
}

function parseControlPacket(buffer) {
  if (buffer.length < 16 || buffer.toString('ascii', 0, 4) !== 'CTL1') {
    return null;
  }

  const version = buffer.readUInt8(4);
  const headerSize = buffer.readUInt8(5);
  const packetSize = buffer.readUInt16LE(6);
  const expectedSize = headerSize + CONTROL_INT_FIELDS.length * 4 +
    CONTROL_FLOAT_FIELDS.length * 4;
  if (version !== 1 || headerSize !== 16 || packetSize !== expectedSize ||
      buffer.length < packetSize) {
    return null;
  }

  const flags0 = buffer.readUInt16LE(12);
  const flags1 = buffer.readUInt16LE(14);
  let offset = headerSize;
  const params = {
    uptime_ms: buffer.readUInt32LE(8),
  };
  CONTROL_INT_FIELDS.forEach((name) => {
    params[name] = buffer.readInt32LE(offset);
    offset += 4;
  });
  CONTROL_FLOAT_FIELDS.forEach((name) => {
    params[name] = buffer.readFloatLE(offset);
    offset += 4;
  });

  params.y_guard_active = Number(Boolean(flags0 & (1 << 0)));
  params.run = Number(Boolean(flags0 & (1 << 1)));
  params.yaw_hold_enabled = Number(Boolean(flags0 & (1 << 2)));
  params.tangent_debug_enabled = Number(Boolean(flags0 & (1 << 3)));
  params.track_tangent_valid = Number(Boolean(flags0 & (1 << 4)));
  params.drive_enabled = Number(Boolean(flags0 & (1 << 5)));
  params.drive_busy = Number(Boolean(flags0 & (1 << 6)));
  params.drive_recognizing = Number(Boolean(flags0 & (1 << 7)));
  params.drive_motion = Number(Boolean(flags0 & (1 << 8)));
  params.drive_test_mode = Number(Boolean(flags0 & (1 << 9)));
  params.drive_brake_test_enabled = Number(Boolean(flags0 & (1 << 10)));
  params.drive_brake_test_holding = Number(Boolean(flags0 & (1 << 11)));
  params.drive_brake_active = Number(Boolean(flags0 & (1 << 12)));
  params.drive_geometry_valid = Number(Boolean(flags0 & (1 << 13)));
  params.drive_view_ready = Number(Boolean(flags0 & (1 << 14)));
  params.red_candidate = Number(Boolean(flags0 & (1 << 15)));
  params.have_target = Number(Boolean(flags1 & (1 << 0)));
  params.hwTest = Number(Boolean(flags1 & (1 << 1)));
  params.camStats = Number(Boolean(flags1 & (1 << 2)));
  params.remote_active = Number(Boolean(flags1 & (1 << 3)));
  const encodedDriveMode = (flags1 >> 4) & 0x3;
  params.drive_mode = encodedDriveMode === 1 || encodedDriveMode === 2
    ? encodedDriveMode
    : (params.drive_enabled ? 1 : 0);

  const stateCode = params.drive_state_code;
  params.drive_state = params.yaw_hold_enabled
    ? 'HEADING_HOLD'
    : (CONTROL_STATE_NAMES[stateCode] || 'UNKNOWN');
  params.drive_abort_reason = CONTROL_ABORT_NAMES[params.drive_abort_reason_code] || 'unknown';
  return params;
}

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
      tangentValid: headerSize >= 56 && Boolean(flags & (1 << 5)),
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
    // 56字节新头部追加切线角度和锚点；旧52字节录像继续解析为无切线。
    tangent: headerSize >= 56 ? {
      valid: Boolean(flags & (1 << 5)),
      angleDeg: buffer.readInt16LE(50) / 100,
      anchor: [buffer.readInt16LE(52), buffer.readInt16LE(54)],
    } : {
      valid: false,
      angleDeg: 0,
      anchor: [-1, -1],
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
  // 录像从第0毫秒就带上当前调参状态，回放开头不会退回前端默认值。
  if (runtime.latestTuning) {
    recordingEvent('tuning', startedAt, runtime.latestTuning);
  }
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
    tuningPackets: runtime.tuningPackets,
    controlPackets: runtime.controlPackets,
    roadPackets: runtime.roadPackets,
    invalidPackets: runtime.invalidPackets,
    lastPacketAt: runtime.lastPacketAt,
    lastControlAt: runtime.lastControlAt,
    lastTuningAt: runtime.lastTuningAt,
    lastRoadAt: runtime.lastRoadAt,
    lastRemote: runtime.lastRemote,
    latestTargetResult: runtime.latestTargetResult,
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
        runtime.jsonPackets += 1;
        if (params.packet_type === 'target_result') {
          const targetResult = parseTargetResultPacket(params);
          if (!targetResult) {
            runtime.invalidPackets += 1;
            return;
          }
          const dedupeKey = `${remote.address}:${targetResult.eventId}:${targetResult.text}`;
          if (runtime.lastTargetResultKey === dedupeKey &&
              receivedAt - runtime.lastTargetResultAt <= TARGET_RESULT_DEDUPE_MS) {
            return;
          }
          runtime.lastTargetResultKey = dedupeKey;
          runtime.lastTargetResultAt = receivedAt;
          runtime.latestTargetResult = { ...targetResult, receivedAt };
          const event = { receivedAt, targetResult };
          broadcast('target_result', event);
          recordingEvent('target_result', receivedAt, targetResult);
          return;
        }
        if (params.packet_type === 'tuning' || params.packet_type === 'camera') {
          const packetType = params.packet_type;
          const tuning = { ...params };
          delete tuning.packet_type;
          runtime.latestTuning = packetType === 'camera'
            ? { ...(runtime.latestTuning || {}), ...tuning }
            : tuning;
          runtime.lastTuningAt = receivedAt;
          runtime.tuningPackets += 1;
          const event = { receivedAt, tuning: runtime.latestTuning };
          broadcast('tuning', event);
          recordingEvent('tuning', receivedAt, runtime.latestTuning);
          return;
        }

        runtime.latestParams = params;
        runtime.lastControlAt = receivedAt;
        const event = { receivedAt, params };
        broadcast('params', event);
        recordingEvent('params', receivedAt, params);
      } catch (error) {
        runtime.invalidPackets += 1;
      }
      return;
    }

    if (message.toString('ascii', 0, 4) === 'CTL1') {
      const params = parseControlPacket(message);
      if (!params) {
        runtime.invalidPackets += 1;
        return;
      }
      runtime.latestParams = params;
      runtime.lastControlAt = receivedAt;
      runtime.controlPackets += 1;
      const event = { receivedAt, params };
      broadcast('params', event);
      recordingEvent('params', receivedAt, params);
      return;
    }

    const road = parseRoadPacket(message);
    if (road) {
      runtime.latestRoad = road;
      runtime.lastRoadAt = receivedAt;
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
            receivedAt: runtime.lastControlAt,
            params: runtime.latestParams,
          })}\n\n`);
        }
        if (runtime.latestTuning) {
          response.write(`event: tuning\ndata: ${JSON.stringify({
            receivedAt: runtime.lastTuningAt,
            tuning: runtime.latestTuning,
          })}\n\n`);
        }
        if (runtime.latestRoad) {
          response.write(`event: road\ndata: ${JSON.stringify({
            receivedAt: runtime.lastRoadAt,
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
      console.log('[提示] 小车DEBUGGER_TARGET_IP应设置为以下地址之一：');
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

module.exports = { parseControlPacket, parseRoadPacket, parseTargetResultPacket, start };
