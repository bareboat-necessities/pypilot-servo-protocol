# pypilot-servo-protocol

Header-only C++ implementation of the pypilot binary UART servo/motor-controller protocol.

This project implements the low-level 4-byte binary protocol used between the pypilot host-side servo driver and the Arduino motor controller. It does not implement pypilot TCP values, NMEA 0183, Signal K, heading control, or AHRS logic.

## Features

- pypilot command and telemetry constants
- CRC-8 compatible packet encoder and decoder
- rolling stream decoder for resynchronizing without a start byte
- host-side API for Linux or Arduino command senders
- controller-side parser/emitter API for Arduino-compatible firmware
- Arduino `Stream` transport
- optional Linux POSIX serial transport
- Linux and Arduino examples
- unit tests with CRC vectors from pypilot-compatible packets

## Packet format

```text
byte 0: command or telemetry code
byte 1: value low byte
byte 2: value high byte
byte 3: CRC-8 over bytes 0..2
```

The payload is little-endian `uint16_t`. CRC uses initial value `0xff` and polynomial `0x31`. There is no sync byte, so receivers scan a rolling 4-byte window and accept only CRC-valid packets.

## Motor command

`COMMAND_CODE = 0xC7`

```text
-1.0 -> 0
 0.0 -> 1000
+1.0 -> 2000
```

## Linux build

```bash
cmake -S . -B build
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

## Linux examples

```bash
./build/pypilot-servo-monitor /dev/ttyUSB0
./build/pypilot-servo-send-command /dev/ttyUSB0 0.1
```

## Arduino examples

The Arduino examples use `Serial1` at 38400 baud. Compile for a board with hardware `Serial1`, such as Arduino Mega or compatible boards.

## Safety

This library can command real steering actuators. Bench-test with the motor disconnected or with a current-limited supply. Applications should send neutral before configuration, set current and temperature limits before motion, resend commands at a controlled rate, send neutral/disengage on shutdown, and stop on stale command input or fault flags.

## License

GPL-3.0-or-later.
