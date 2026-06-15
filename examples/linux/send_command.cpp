#include <chrono>
#include <cstdlib>
#include <iostream>
#include <thread>
#include <pypilot_servo_protocol.hpp>

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "usage: " << argv[0] << " /dev/ttyUSB0 command[-1..1]\n";
        return 2;
    }
    pypilot_servo_protocol::LinuxSerialTransport serial;
    if (!serial.open_device(argv[1], 38400)) {
        std::cerr << "failed to open " << argv[1] << "\n";
        return 1;
    }
    pypilot_servo_protocol::Host host(serial);
    host.force_unsync();
    host.set_max_current_amps(4.5f);
    host.set_max_controller_temp_c(60.0f);
    host.set_max_motor_temp_c(60.0f);
    const float command = static_cast<float>(std::atof(argv[2]));
    for (int i = 0; i < 10; ++i) {
        host.send_command(command);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    host.neutral();
    host.disengage();
    return 0;
}
