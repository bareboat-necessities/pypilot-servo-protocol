#include <cassert>
#include <cmath>
#include <stdint.h>
#include <pypilot_servo_protocol.hpp>

using namespace pypilot_servo_protocol;

int main() {
    const uint8_t neutral[3] = {COMMAND_CODE, 0xE8, 0x03};
    const uint8_t reverse[3] = {COMMAND_CODE, 0x00, 0x00};
    const uint8_t forward[3] = {COMMAND_CODE, 0xD0, 0x07};
    const uint8_t reset[3] = {RESET_CODE, 0x00, 0x00};
    assert(crc8(neutral, 3) == 0x9D);
    assert(crc8(reverse, 3) == 0x8F);
    assert(crc8(forward, 3) == 0xDC);
    assert(crc8(reset, 3) == 0xF6);

    RawPacket raw = encode_packet(COMMAND_CODE, 1000);
    assert(raw.bytes[0] == COMMAND_CODE);
    assert(raw.bytes[1] == 0xE8);
    assert(raw.bytes[2] == 0x03);
    assert(raw.bytes[3] == 0x9D);

    Packet p{};
    assert(decode_packet(raw.bytes, p));
    assert(p.code == COMMAND_CODE);
    assert(p.value == 1000);

    RawPacket flags = encode_packet(FLAGS_CODE, SYNC_FLAG | ENGAGED_FLAG);
    const uint8_t stream[] = {0x55, 0xaa, flags.bytes[0], flags.bytes[1], flags.bytes[2], flags.bytes[3]};
    Decoder decoder;
    bool got = false;
    for (uint8_t b : stream) got = decoder.push(b, p) || got;
    assert(got);
    assert(p.code == FLAGS_CODE);

    assert(encode_command(-1.0f) == 0);
    assert(encode_command(0.0f) == 1000);
    assert(encode_command(1.0f) == 2000);
    assert(std::fabs(decode_command_value(1500) - 0.5f) < 0.001f);

    float r = 1.0f;
    assert(decode_rudder(encode_rudder(0.0f), r));
    assert(std::fabs(r) < 0.001f);
    assert(!decode_rudder(65535, r));

    Telemetry t;
    assert(t.apply(Packet{CURRENT_CODE, 123}));
    assert(t.has_current);
    assert(std::fabs(t.current_a - 1.23f) < 0.001f);
    assert(t.apply(Packet{FLAGS_CODE, static_cast<uint16_t>(SYNC_FLAG | OVERTEMP_FAULT)}));
    assert(t.synced());
    assert(t.faulted());
    return 0;
}
