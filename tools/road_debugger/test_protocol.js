'use strict';

const assert = require('assert');
const { parseRoadPacket } = require('./server');

function makePacket() {
  const left = [[10, 200], [20, 160]];
  const center = [[160, 220], [160, 140], [160, 80]];
  const right = [[300, 200]];
  const packet = Buffer.alloc(52 + (left.length + center.length + right.length) * 4);
  packet.write('RDL1', 0, 'ascii');
  packet.writeUInt8(1, 4);
  packet.writeUInt8(0b11111, 5);
  packet.writeUInt16LE(52, 6);
  packet.writeUInt32LE(42, 8);
  packet.writeUInt32LE(1234, 12);
  packet.writeUInt16LE(320, 16);
  packet.writeUInt16LE(240, 18);
  packet.writeUInt8(left.length, 20);
  packet.writeUInt8(center.length, 21);
  packet.writeUInt8(right.length, 22);
  packet.writeInt8(2, 23);
  [60, 120, 20, 12, 58, 96, 24, 24, 160, 120, 70, 80, 90, 0].forEach((value, index) => {
    packet.writeInt16LE(value, 24 + index * 2);
  });
  let offset = 52;
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
assert.strictEqual(parseRoadPacket(Buffer.from('bad')), null);

console.log('RDL1 protocol test passed');
