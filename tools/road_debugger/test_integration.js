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

function makeControlPacket() {
  const packet = Buffer.alloc(212);
  packet.write('CTL1', 0, 'ascii');
  packet.writeUInt8(1, 4);
  packet.writeUInt8(16, 5);
  packet.writeUInt16LE(packet.length, 6);
  packet.writeUInt32LE(500, 8);
  packet.writeUInt16LE((1 << 1) | (1 << 5) | (1 << 6) | (1 << 12), 12);
  packet.writeUInt16LE(1, 14);
  const integers = [3, 5, 7, 35, 6, 0, -7000, 40, 2, 2, 61, 4, 0, 0, 0, 2];
  const floats = [
    20.1, 19.9, 2.5, 20, 20, -12, -10, 3, 4, 1, 1.5,
    180, 170, 12, 420, 10.2, 8, 0, 24, 25, 1, 0, 0, 15,
    0.5, 0.2, 0.1, 180, 7, 1.2, 2.4, 0.25, 0.5,
  ];
  let offset = 16;
  integers.forEach((value) => {
    packet.writeInt32LE(value, offset);
    offset += 4;
  });
  floats.forEach((value) => {
    packet.writeFloatLE(value, offset);
    offset += 4;
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
      run: 0,
      encoder1_speed_avg: 10.1,
      encoder2_speed_avg: 9.9,
      camera_process_avg_ms: 16.4,
      camera_fps: 59.7,
      have_target: 1,
      item_flag: 1,
    })));
    await sendUdp(makeControlPacket());
    await sendUdp(Buffer.from(JSON.stringify({
      packet_type: 'tuning',
      carProfile: 1,
      profileSpd: 45,
      P: 454,
      I: 14,
      yGuardDps: 500,
      dbUseTangent: 1,
      dbTurnAngle: 25,
      dbReturnBias: 46,
      dbPassDist: 0.14,
      dbHMax: 200,
      dbHTol: 2,
      dbRecoverDps: 55,
      dbBrakePwm: 6000,
      dbEarlyBrake: 1,
      yawHoldRMax: 10,
      udp: 2,
      vofa: 0,
      hwTest: 0,
      hwPwm: 0,
      camStats: 1,
      udp_control_send_fail_total: 2,
      udp_road_send_fail_total: 1,
    })));
    await sendUdp(Buffer.from(JSON.stringify({
      packet_type: 'camera',
      camera_process_last_ms: 17.2,
      camera_process_avg_ms: 16.8,
      camera_process_max_ms: 23.5,
      camera_fps: 59.4,
      camera_process_overrun_count: 1,
    })));
    await sendUdp(makeRoadPacket());
    await delay(120);

    const statusResponse = await request('GET', '/api/status');
    assert.strictEqual(statusResponse.status, 200);
    assert.strictEqual(statusResponse.json.jsonPackets, 3);
    assert.strictEqual(statusResponse.json.tuningPackets, 2);
    assert.strictEqual(statusResponse.json.controlPackets, 1);
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
    assert(pageResponse.text.includes('earlyBrakeCommand'));
    assert(pageResponse.text.includes('brakeTestCommand'));
    assert(pageResponse.text.includes('headingHoldCommand'));
    assert(pageResponse.text.includes('tangentDebugCommand'));
    assert(pageResponse.text.includes('cameraStatsCommand'));
    assert(pageResponse.text.includes('cameraStatsStrip'));
    assert(pageResponse.text.includes('cameraProcessAverage'));
    assert(pageResponse.text.includes('#spd=35;'));
    assert(pageResponse.text.includes('#spd=0;'));
    assert(pageResponse.text.includes('#udp=0;'));
    assert(pageResponse.text.includes('#udp=2;'));
    assert(!pageResponse.text.includes('#spd=20;'));
    assert(!pageResponse.text.includes('#udp=1;'));
    assert(pageResponse.text.includes('remoteUp'));
    assert(pageResponse.text.includes('remoteDown'));
    assert(pageResponse.text.includes('remoteLeft'));
    assert(pageResponse.text.includes('remoteRight'));
    assert(pageResponse.text.includes('左转 160dps'));
    assert(pageResponse.text.includes('右转 160dps'));
    assert(!pageResponse.text.includes('转 60dps'));
    assert(pageResponse.text.includes('tuningControls'));
    assert(pageResponse.text.includes('tuningSnapshotTime'));
    assert(pageResponse.text.includes('stableProfileCommand'));
    assert(pageResponse.text.includes('proProfileCommand'));
    assert(pageResponse.text.includes('profileBadge'));
    assert(pageResponse.text.includes('timeoutBadge'));
    assert(!pageResponse.text.includes('parameterList'));
    assert(!pageResponse.text.includes('detectEveryFrameButton'));
    assert(!pageResponse.text.includes('detectEveryTwoFramesButton'));

    const appResponse = await request('GET', '/app.js');
    assert.strictEqual(appResponse.status, 200);
    const htmlIds = new Set([...pageResponse.text.matchAll(/id="([^"]+)"/g)]
      .map((match) => match[1]));
    const missingDomIds = [...appResponse.text.matchAll(/\$\('([^']+)'\)/g)]
      .map((match) => match[1])
      .filter((id) => !htmlIds.has(id));
    assert.deepStrictEqual([...new Set(missingDomIds)], []);
    const tuningKeys = [
      'profileSpd', 'P', 'I', 'D', 'dirP', 'dirD', 'AIM', 'spd_slow_ratio', 'begin_x',
      'gDbg', 'gTar', 'gOP', 'gOD', 'gIP', 'gII', 'gTMax', 'gRMax', 'yGuardDps',
      'yGuardBase', 'yGuardRMax', 'gSign', 'tSign', 'dbUseTangent', 'dbRecSpd',
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
    assert(appResponse.text.includes("key: 'yGuardDps', label: '瞄点越界救车角速度', min: 0, max: 700, step: 5, unit: 'dps', defaultValue: 500, hardMax: true"));
    assert(appResponse.text.includes("key: 'profileSpd', label: '巡线目标速度', min: 0, max: 60, step: 0.5, unit: 'RPS', defaultValue: 35"));
    assert(appResponse.text.includes("key: 'yGuardBase', label: '救车前进基准速度'"));
    assert(appResponse.text.includes("key: 'yGuardRMax', label: '救车差速上限'"));
    assert(appResponse.text.includes("'profileSpd', 'P', 'I', 'D'"));
    assert(appResponse.text.includes("control.label.textContent = `${control.config.label}${profileScoped && proMode ? '-pro' : ''}`"));
    assert(appResponse.text.includes("#carProfile=${normalizedMode};"));
    assert(appResponse.text.includes("const TUNING_MAXES_STORAGE_KEY = 'tuningSliderMaxes'"));
    assert(appResponse.text.includes("maxEditor.className = 'tuning-max-editor'"));
    assert(appResponse.text.includes("maxLabel.textContent = '上限'"));
    assert(appResponse.text.includes("key: 'dbUseTangent', label: '目标处切线参考', kind: 'toggle', defaultValue: 0"));
    assert(appResponse.text.includes("key: 'dbRecSpd', label: '识别阶段前进基准速度', min: 0, max: 40, step: 0.5, unit: 'RPS', defaultValue: 0"));
    assert(appResponse.text.includes("key: 'dbTurnAngle', label: '向外转角', min: 0, max: 90, step: 1, unit: 'deg', defaultValue: 42"));
    assert(appResponse.text.includes("key: 'dbReturnBias', label: '回赛道预偏角', min: 0, max: 91, step: 1, unit: 'deg', defaultValue: 30"));
    assert(appResponse.text.includes("key: 'dbPassDist', label: '最短斜行距离', min: 0, max: 2, step: 0.01, unit: 'm', defaultValue: 0.32"));
    assert(appResponse.text.includes("key: 'dbSafeDist', label: '目标后安全余量', min: 0, max: 1, step: 0.01, unit: 'm', defaultValue: 0"));
    assert(appResponse.text.includes("key: 'dbTurnRps', label: '转出阶段前进基准速度', min: 0, max: 40, step: 1, unit: 'RPS', defaultValue: 0"));
    assert(appResponse.text.includes("key: 'dbForwardRps', label: '斜行阶段前进基准速度', min: 0, max: 40, step: 1, unit: 'RPS', defaultValue: 10"));
    assert(appResponse.text.includes("key: 'dbExitRps', label: '转入阶段前进基准速度', min: 0, max: 40, step: 1, unit: 'RPS', defaultValue: 10"));
    assert(appResponse.text.includes("key: 'dbViewMax', label: '最大观察夹角', min: 0, max: 90, step: 1, unit: 'deg', defaultValue: 46"));
    assert(appResponse.text.includes("key: 'dbHTol', label: '航向允许误差（退出+1°）', min: 0, max: 10, step: 0.1, unit: 'deg', defaultValue: 4.5"));
    assert(appResponse.text.includes("key: 'dbBrakePwm', label: '主动制动反向PWM', min: 0, max: 9000, step: 50, defaultValue: 9000, hardMax: true"));
    assert(appResponse.text.includes("key: 'dbBrakeRelease', label: '制动释放速度', min: 0, max: 200, step: 0.5, unit: 'RPS', defaultValue: 0"));
    assert(appResponse.text.includes("key: 'dbBrakeTimeout', label: '制动超时', min: 1, max: 2000, step: 10, unit: 'ms', defaultValue: 511"));
    assert(appResponse.text.includes("key: 'hwTest', label: 'PWM1硬件测试', kind: 'toggle', defaultValue: 0"));
    assert(appResponse.text.includes("key: 'hwPwm', label: 'PWM1正向占空比', min: 0, max: 7000, step: 50, defaultValue: 0, hardMax: true"));
    assert(appResponse.text.includes("#remote=0;"));
    assert(appResponse.text.includes("#yawHold="));
    assert(appResponse.text.includes("#tangentDbg="));
    assert(appResponse.text.includes("#test_brake="));
    assert(appResponse.text.includes('#dbEarlyBrake=${enabled ? 0 : 1};'));
    assert(appResponse.text.includes('PRO首次候选刹车：固定开启'));
    assert(appResponse.text.includes("#drive=${enabled ? 0 : 1};"));
    assert(appResponse.text.includes("#camStats=${enabled ? 0 : 1};"));
    assert(appResponse.text.includes('camera_process_avg_ms'));
    assert(appResponse.text.includes('renderCameraPerformance'));
    assert(appResponse.text.includes('TIMEOUT_SOURCE_NAMES'));
    assert(appResponse.text.includes('telemetryMode'));
    assert(appResponse.text.includes('controlAge <= 100'));
    assert(appResponse.text.includes('controlAge <= 500'));
    assert(!appResponse.text.includes("key: 'spd'"));
    assert(!appResponse.text.includes("key: 'gyro'"));
    assert(!appResponse.text.includes("key: 'dbMode'"));
    assert(!appResponse.text.includes("key: 'dbSideMs'"));
    assert(!appResponse.text.includes('#dbDetectEvery='));
    assert(!appResponse.text.includes('PINNED_PARAMETERS_STORAGE_KEY'));
    assert(appResponse.text.includes('params.drive_brake_test_enabled'));
    assert(appResponse.text.includes('sendRemoteHeartbeat'));

    const mainSource = fs.readFileSync(path.join(__dirname, '..', '..', 'main', 'main.cpp'), 'utf8');
    const hardwareTestSource = fs.readFileSync(path.join(__dirname, '..', '..', 'main', 'hardware_test.cpp'), 'utf8');
    const frontUiSource = fs.readFileSync(path.join(__dirname, '..', '..', 'main', 'front_ui.cpp'), 'utf8');
    const profileSource = fs.readFileSync(path.join(__dirname, '..', '..', 'main', 'control_profile.cpp'), 'utf8');
    const dirPdSource = fs.readFileSync(path.join(__dirname, '..', '..', 'example', 'src', 'dir_pd.cpp'), 'utf8');
    const driveBySource = fs.readFileSync(path.join(__dirname, '..', '..', 'example', 'src', 'drive_by.cpp'), 'utf8');
    const stableDefaultsSource = profileSource.slice(
      profileSource.indexOf('ControlProfileValues make_stable_defaults()'),
      profileSource.indexOf('ControlProfileValues make_pro_defaults()'),
    );
    const speedControlSource = driveBySource.slice(
      driveBySource.indexOf('bool drive_by_speed_control_update()'),
      driveBySource.indexOf('bool drive_by_start_test('),
    );
    assert(mainSource.includes('constexpr int kRoadTelemetryMinIntervalMs = 20;'));
    assert(mainSource.includes('constexpr int kControlTelemetryPacketSize'));
    assert(mainSource.includes('kControlTelemetryPacketSize < 256'));
    assert(mainSource.includes('{\\"packet_type\\":\\"camera\\",'));
    assert(mainSource.includes('udp_control_send_fail_total'));
    assert(mainSource.includes('udp_road_send_fail_total'));
    assert(!mainSource.includes('red_pre_last_ms'));
    assert(!mainSource.includes('red_pre_avg_ms'));
    assert(mainSource.includes('vision_y_guard_target_dps = 500.0f;'));
    assert(mainSource.includes('if (ftmp > 700.0f) ftmp = 700.0f;'));
    assert(mainSource.includes('\\"carProfile\\":%d,\\"profileSpd\\":%.2f'));
    assert(mainSource.includes('#carProfile=%d;'));
    assert(mainSource.includes('#profileSpd=%f;'));
    assert(hardwareTestSource.includes('constexpr int kHardwareTestMaxPwm = 7000;'));
    assert(frontUiSource.includes('constexpr float kRemoteYawRateDps = 160.0f;'));
    assert(frontUiSource.includes('constexpr auto kPhysicalStartDelay = std::chrono::milliseconds(1000);'));
    assert(frontUiSource.includes('physical_start_deadline = now + kPhysicalStartDelay;'));
    assert(frontUiSource.includes('void front_ui_start()'));
    assert(frontUiSource.includes('physical_start_pending = false;\n    start_car();'));
    assert(frontUiSource.includes('K0 TARGET K1 PROFILE'));
    assert(frontUiSource.includes('control_profile_switch(requested_mode)'));
    assert(frontUiSource.includes('已切换为%s模式'));
    assert(frontUiSource.includes('超载（OVERLOAD）'));
    assert(!frontUiSource.includes('PRO mode ignores K1 speed selection'));
    assert(dirPdSource.includes('constexpr float kVisionYGuardMaxDps = 700.0f;'));
    assert(dirPdSource.includes('vision_y_guard_turn_max_rps'));
    assert(dirPdSource.includes('vision_y_guard_base_rps'));
    assert(profileSource.includes('ControlProfile stable_profile'));
    assert(profileSource.includes('ControlProfile pro_profile'));
    ['35.0f', '454.0f', '14.0f', '0.128f', '1.55f', '0.25f'].forEach((value) => {
      assert(stableDefaultsSource.includes(value));
    });
    assert(profileSource.includes('values.target_speed_rps = 45.0f;'));
    assert(profileSource.includes('values.direction_p = 0.231f;'));
    assert(profileSource.includes('values.direction_d = 3.5f;'));
    assert(profileSource.includes('values.aim_m = 0.40f;'));
    assert(profileSource.includes('values.speed_slow_ratio = 40;'));
    assert(profileSource.includes('values.rescue_target_dps = 650.0f;'));
    assert(profileSource.includes('values.rescue_turn_max_rps = 30.0f;'));
    assert(driveBySource.includes('g_candidate_brake_active'));
    assert(driveBySource.includes('g_stable_early_brake_enabled = false'));
    assert(driveBySource.includes('control_profile_is_pro() ||'));
    assert(driveBySource.includes('g_stable_early_brake_enabled;'));
    assert(driveBySource.includes('start_candidate_brake();'));
    assert(driveBySource.includes('if (g_candidate_brake_active && !g_brake_active)'));
    assert(driveBySource.includes('g_drive_by_busy || g_candidate_brake_active'));
    assert(mainSource.includes('#dbEarlyBrake=%d;'));
    assert(mainSource.includes('\\\"dbEarlyBrake\\\":%d'));
    assert(mainSource.includes('drive_by_stable_early_brake_set_enable(itmp != 0)'));
    assert(mainSource.includes('const bool zebra_detection_armed = car_running &&'));
    assert(mainSource.includes('constexpr int kZebraDetectionFrameInterval = 3;'));
    assert(mainSource.includes('++zebra_detection_frame_counter >='));
    assert(mainSource.includes('std::chrono::seconds(5)'));
    assert(mainSource.includes('check_is_zebra(&img_raw, x, y, thres)'));
    assert(mainSource.includes('[斑马线] 检测到斑马线，停车'));
    assert(mainSource.includes('if (zebra_detected) {\n            printf'));
    assert(!mainSource.includes('std::chrono::seconds(3)&&car_running==1'));
    assert.strictEqual((driveBySource.match(/control_profile_is_pro\(\)/g) || []).length, 2);
    assert.strictEqual((driveBySource.match(/g_candidate_brake_active = true;/g) || []).length, 1);
    assert(!speedControlSource.includes('control_profile_is_pro()'));
    assert(profileSource.includes('!front_ui_is_running()'));
    assert.strictEqual((dirPdSource.match(/\[救车\] 触发救车/g) || []).length, 1);

    const stopResponse = await request('POST', '/api/recording/stop', {});
    assert.strictEqual(stopResponse.status, 200);
    assert.strictEqual(stopResponse.json.recording.eventCount, 5);

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
    assert(recordingText.includes('"camera_process_avg_ms":16.4'));
    assert(recordingText.includes('"to_id":5'));
    assert(recordingText.includes('"to_used":10.199999809265137'));
    assert(recordingText.includes('"type":"road"'));
    assert(recordingText.includes('"type":"tuning"'));
    assert(recordingText.includes('"carProfile":1'));
    assert(recordingText.includes('"profileSpd":45'));
    assert(recordingText.includes('"dbUseTangent":1'));
    assert(recordingText.includes('"dbEarlyBrake":1'));
    assert(recordingText.includes('"dbHTol":2'));
    assert(recordingText.includes('"yawHoldRMax":10'));
    assert(recordingText.includes('"hwTest":0'));
    assert(recordingText.includes('"hwPwm":0'));
    assert(recordingText.includes('"camStats":1'));
    assert(recordingText.includes('"camera_process_avg_ms":16.8'));
    assert(recordingText.includes('"udp_control_send_fail_total":2'));
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
