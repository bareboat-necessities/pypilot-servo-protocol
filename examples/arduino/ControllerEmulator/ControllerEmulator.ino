#include <Arduino.h>
#include <pypilot_servo_protocol.hpp>

pypilot_servo_protocol::Decoder decoder;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("paste 4-byte packet bytes on Serial to test decoder");
}

void loop() {
  while (Serial.available()) {
    pypilot_servo_protocol::Packet packet;
    int b = Serial.read();
    if (decoder.push((uint8_t)b, packet)) {
      Serial.print("code=0x");
      Serial.print(packet.code, HEX);
      Serial.print(" value=");
      Serial.println(packet.value);
    }
  }
}
