#include <Arduino.h>
#include <pypilot_servo_protocol.hpp>

void printPacket(const pypilot_servo_protocol::RawPacket& packet) {
  for (int i = 0; i < 4; ++i) {
    if (packet.bytes[i] < 16) Serial.print('0');
    Serial.print(packet.bytes[i], HEX);
    if (i != 3) Serial.print(' ');
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Serial.println("pypilot servo protocol packet examples");
  printPacket(pypilot_servo_protocol::encode_packet(
    pypilot_servo_protocol::COMMAND_CODE,
    pypilot_servo_protocol::encode_command(0.0f)));
  printPacket(pypilot_servo_protocol::encode_packet(
    pypilot_servo_protocol::DISENGAGE_CODE, 0));
}

void loop() {
}
