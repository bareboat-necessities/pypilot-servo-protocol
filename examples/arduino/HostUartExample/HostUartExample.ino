#include <Arduino.h>
#include <pypilot_servo_protocol.hpp>

pypilot_servo_protocol::ArduinoStreamTransport transport(Serial1);
pypilot_servo_protocol::Host servo(transport);
pypilot_servo_protocol::Telemetry telemetry;

unsigned long lastCommandMs = 0;

void setup() {
  Serial.begin(115200);
  Serial1.begin(38400);

  // Put the controller into a known safe state before any other command.
  servo.force_unsync();
  servo.neutral();
  servo.set_max_current_amps(4.5f);
  servo.set_max_controller_temp_c(60.0f);
  servo.set_max_motor_temp_c(60.0f);
}

void loop() {
  while (servo.poll(telemetry)) {
    if (telemetry.has_flags) {
      Serial.print("flags=0x");
      Serial.println(telemetry.flags, HEX);
    }
    if (telemetry.has_current) {
      Serial.print("current_a=");
      Serial.println(telemetry.current_a, 2);
    }
    if (telemetry.has_voltage) {
      Serial.print("voltage_v=");
      Serial.println(telemetry.voltage_v, 2);
    }
  }

  if (millis() - lastCommandMs >= 100) {
    lastCommandMs = millis();

    // Bench-safe default: hold neutral. Replace only after wiring and limits are verified.
    servo.send_command(0.0f);
  }
}
