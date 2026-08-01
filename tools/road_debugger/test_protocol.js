'use strict';

const assert = require('assert');
const {
  parseControlPacket,
  parseRoadPacket,
  parseTargetResultPacket,
} = require('./server');

function makeControlPacket() {
  const packet = Buffer.alloc(212);
  packet.write('CTL1', 0, 'ascii');
  packet.writeUInt8(1, 4);
  packet.writeUInt8(16, 5);
  packet.writeUInt16LE(packet.length, 6);
  packet.writeUInt32LE(4321, 8);
  packet.writeUInt16LE(0xffff & ~(1 << 2), 12);
  packet.writeUInt16LE(0b10_1111, 14);

  const integers = [8, 5, 13, 35, 6, 5, -7000, 42, 2, 2, 123, 4, 1, 2, 3, 2];
  const floats = Array.from({ length: 33 }, (_, index) => index + 0.25);
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

function makePacket(headerSize = 56) {
  const left = [[10, 200], [20, 160]];
  const center = [[160, 220], [160, 140], [160, 80]];
  const right = [[300, 200]];
  const packet = Buffer.alloc(headerSize + (left.length + center.length + right.length) * 4);
  packet.write('RDL1', 0, 'ascii');
  packet.writeUInt8(1, 4);
  packet.writeUInt8(headerSize >= 56 ? 0b111111 : 0b11111, 5);
  packet.writeUInt16LE(headerSize, 6);
  packet.writeUInt32LE(42, 8);
  packet.writeUInt32LE(1234, 12);
  packet.writeUInt16LE(320, 16);
  packet.writeUInt16LE(240, 18);
  packet.writeUInt8(left.length, 20);
  packet.writeUInt8(center.length, 21);
  packet.writeUInt8(right.length, 22);
  packet.writeInt8(2, 23);
  [60, 120, 20, 12, 58, 96, 24, 24, 160, 120, 70, 80, 90, 1234].forEach((value, index) => {
    packet.writeInt16LE(value, 24 + index * 2);
  });
  if (headerSize >= 56) {
    packet.writeInt16LE(155, 52);
    packet.writeInt16LE(118, 54);
  }
  let offset = headerSize;
  [...left, ...center, ...right].forEach(([x, y]) => {
    packet.writeInt16LE(x, offset);
    packet.writeInt16LE(y, offset + 2);
    offset += 4;
  });
  return packet;
}

const parsed = parseRoadPacket(makePacket());
assert(parsed);
assert.strictEqual(parsed.sequence, 42);
assert.strictEqual(parsed.carTimeMs, 1234);
assert.strictEqual(parsed.itemFlag, 2);
assert.deepStrictEqual(parsed.aim, [160, 120]);
assert.deepStrictEqual(parsed.sourceCounts, { left: 70, center: 80, right: 90 });
assert.deepStrictEqual(parsed.lines.left, [[10, 200], [20, 160]]);
assert.deepStrictEqual(parsed.lines.center, [[160, 220], [160, 140], [160, 80]]);
assert.deepStrictEqual(parsed.lines.right, [[300, 200]]);
assert.strictEqual(parsed.flags.running, true);
assert.strictEqual(parsed.flags.tangentValid, true);
assert.deepStrictEqual(parsed.tangent, {
  valid: true,
  angleDeg: 12.34,
  anchor: [155, 118],
});

const legacy = parseRoadPacket(makePacket(52));
assert(legacy);
assert.deepStrictEqual(legacy.tangent, {
  valid: false,
  angleDeg: 0,
  anchor: [-1, -1],
});
assert.strictEqual(parseRoadPacket(Buffer.from('bad')), null);

assert.deepStrictEqual(parseTargetResultPacket({
  packet_type: 'target_result',
  event_id: 17,
  result: 0,
  text: '[目标板] 类型=武器（weapon）',
}), {
  eventId: 17,
  result: 0,
  text: '[目标板] 类型=武器（weapon）',
});
assert.strictEqual(parseTargetResultPacket({
  packet_type: 'target_result',
  event_id: 17,
  result: 4,
  text: 'bad',
}), null);

const control = parseControlPacket(makeControlPacket());
assert(control);
assert.strictEqual(control.uptime_ms, 4321);
assert.strictEqual(control.gyro_timeout, 8);
assert.strictEqual(control.to_id, 5);
assert.strictEqual(control.to_total, 13);
assert.strictEqual(control.selected_speed, 35);
assert.strictEqual(control.drive_state, 'TURN_OUT');
assert.strictEqual(control.drive_abort_reason, 'phase_timeout');
assert.strictEqual(control.drive_brake_pwm, -7000);
assert.strictEqual(control.item_flag, 2);
assert.strictEqual(control.encoder1_speed_avg, 0.25);
assert.strictEqual(control.gyro_target_dps, 11.25);
assert.strictEqual(control.drive_test_target_distance_m, 32.25);
assert.strictEqual(control.run, 1);
assert.strictEqual(control.drive_enabled, 1);
assert.strictEqual(control.drive_mode, 2);
assert.strictEqual(control.have_target, 1);
assert.strictEqual(control.camStats, 1);
assert.strictEqual(control.remote_active, 1);
const legacyControlPacket = makeControlPacket();
legacyControlPacket.writeUInt16LE(0b1111, 14);
assert.strictEqual(parseControlPacket(legacyControlPacket).drive_mode, 1);
assert.strictEqual(parseControlPacket(Buffer.from('bad')), null);
const invalidControl = makeControlPacket();
invalidControl.writeUInt16LE(211, 6);
assert.strictEqual(parseControlPacket(invalidControl), null);

console.log('CTL1 and RDL1 protocol tests passed');
