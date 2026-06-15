#include <chrono>
#include <iostream>
#include <thread>
#include <pypilot_servo_protocol.hpp>

int main(int argc, char** argv) {
    const char* device = argc > 1 ? argv[1] : "/dev/ttyUSB0";
    pypilot_servo_protocol::LinuxSerialTransport serial;
    if (!serial.open_device(device, 38400)) {
        std::cerr << "failed to open " << device << "\n";
        return 1;
    }

    pypilot_servo_protocol::Host host(serial);
    pypilot_servo_protocol::Telemetry telemetry;
    host.force_unsync();
    host.neutral();

    while (true) {
        host.neutral();
        for (int i = 0; i < 50; ++i) {
            if (host.poll(telemetry)) {
                if (telemetry.has_flags) {
                    std::cout << "flags=0x" << std::hex << telemetry.flags << std::dec;
                    if (telemetry.synced()) std::cout << " synced";
                    if (telemetry.engaged()) std::cout << " engaged";
                    if (telemetry.faulted()) std::cout << " fault";
                    std::cout << "\n";
                }
                if (telemetry.has_current) std::cout << "current=" << telemetry.current_a << " A\n";
                if (telemetry.has_voltage) std::cout << "voltage=" << telemetry.voltage_v << " V\n";
                if (telemetry.has_rudder) std::cout << (telemetry.rudder_valid ? "rudder=" : "rudder=invalid") << (telemetry.rudder_valid ? telemetry.rudder : 0.0f) << "\n";
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}
