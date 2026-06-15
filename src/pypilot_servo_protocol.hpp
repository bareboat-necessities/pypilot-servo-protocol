#pragma once

#include <stddef.h>
#include <stdint.h>

namespace pypilot_servo_protocol {

enum CommandCode : uint8_t {
    COMMAND_CODE = 0xC7,
    RESET_CODE = 0xE7,
    MAX_CURRENT_CODE = 0x1E,
    MAX_CONTROLLER_TEMP_CODE = 0xA4,
    MAX_MOTOR_TEMP_CODE = 0x5A,
    RUDDER_RANGE_CODE = 0xB6,
    RUDDER_MIN_CODE = 0x2B,
    RUDDER_MAX_CODE = 0x4D,
    REPROGRAM_CODE = 0x19,
    DISENGAGE_CODE = 0x68,
    MAX_SLEW_CODE = 0x71,
    EEPROM_READ_CODE = 0x91,
    EEPROM_WRITE_CODE = 0x53,
    CLUTCH_PWM_AND_BRAKE_CODE = 0x36
};

enum TelemetryCode : uint8_t {
    CURRENT_CODE = 0x1C,
    VOLTAGE_CODE = 0xB3,
    CONTROLLER_TEMP_CODE = 0xF9,
    MOTOR_TEMP_CODE = 0x48,
    RUDDER_SENSE_CODE = 0xA7,
    FLAGS_CODE = 0x8F,
    EEPROM_VALUE_CODE = 0x9A
};

enum ControllerFlag : uint16_t {
    SYNC_FLAG = 1u,
    OVERTEMP_FAULT = 2u,
    OVERCURRENT_FAULT = 4u,
    ENGAGED_FLAG = 8u,
    INVALID_PACKET_FLAG = 16u,
    PORT_PIN_FAULT = 32u,
    STARBOARD_PIN_FAULT = 64u,
    BADVOLTAGE_FAULT = 128u,
    MIN_RUDDER_FAULT = 256u,
    MAX_RUDDER_FAULT = 512u,
    CURRENT_RANGE_FLAG = 1024u,
    BAD_FUSES_FLAG = 2048u,
    REBOOTED_FLAG = 32768u
};

struct Packet { uint8_t code; uint16_t value; };
struct RawPacket { uint8_t bytes[4]; };

inline bool is_fault_flag(uint16_t flags) {
    return (flags & (OVERTEMP_FAULT | OVERCURRENT_FAULT | INVALID_PACKET_FLAG |
                     PORT_PIN_FAULT | STARBOARD_PIN_FAULT | BADVOLTAGE_FAULT |
                     MIN_RUDDER_FAULT | MAX_RUDDER_FAULT | BAD_FUSES_FLAG)) != 0;
}

inline uint8_t crc8(const uint8_t* data, uint8_t len) {
    uint8_t crc = 0xffu;
    for (uint8_t i = 0; i < len; ++i) {
        crc = static_cast<uint8_t>(crc ^ data[i]);
        for (uint8_t bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x80u) ? static_cast<uint8_t>((crc << 1) ^ 0x31u)
                                : static_cast<uint8_t>(crc << 1);
        }
    }
    return crc;
}

inline RawPacket encode_packet(uint8_t code, uint16_t value) {
    RawPacket p = {{0, 0, 0, 0}};
    p.bytes[0] = code;
    p.bytes[1] = static_cast<uint8_t>(value & 0xffu);
    p.bytes[2] = static_cast<uint8_t>((value >> 8) & 0xffu);
    p.bytes[3] = crc8(p.bytes, 3);
    return p;
}

inline bool decode_packet(const uint8_t bytes[4], Packet& out) {
    if (crc8(bytes, 3) != bytes[3]) return false;
    out.code = bytes[0];
    out.value = static_cast<uint16_t>(bytes[1]) |
                static_cast<uint16_t>(static_cast<uint16_t>(bytes[2]) << 8);
    return true;
}

inline uint16_t encode_command(float normalized) {
    if (normalized < -1.0f) normalized = -1.0f;
    if (normalized > 1.0f) normalized = 1.0f;
    return static_cast<uint16_t>((normalized + 1.0f) * 1000.0f + 0.5f);
}

inline float decode_command_value(uint16_t value) {
    return static_cast<float>(value) / 1000.0f - 1.0f;
}

inline uint16_t encode_rudder(float normalized) {
    if (normalized < -0.5f) normalized = -0.5f;
    if (normalized > 0.5f) normalized = 0.5f;
    return static_cast<uint16_t>((normalized + 0.5f) * 65472.0f + 0.5f);
}

inline bool decode_rudder(uint16_t value, float& out_normalized) {
    if (value == 65535u) { out_normalized = 0.0f; return false; }
    out_normalized = static_cast<float>(value) / 65472.0f - 0.5f;
    return true;
}

inline uint16_t encode_centivalue(float value) {
    if (value > 327.67f) value = 327.67f;
    if (value < -327.68f) value = -327.68f;
    const int16_t v = static_cast<int16_t>(value * 100.0f + (value >= 0.0f ? 0.5f : -0.5f));
    return static_cast<uint16_t>(v);
}

inline float decode_unsigned_centivalue(uint16_t value) { return static_cast<float>(value) / 100.0f; }
inline float decode_signed_centivalue(uint16_t value) { return static_cast<float>(static_cast<int16_t>(value)) / 100.0f; }

class Decoder {
public:
    Decoder() { reset(); }
    void reset() { count_ = 0; window_[0] = window_[1] = window_[2] = window_[3] = 0; }
    bool push(uint8_t byte, Packet& out) {
        if (count_ < 4) window_[count_++] = byte;
        else { window_[0] = window_[1]; window_[1] = window_[2]; window_[2] = window_[3]; window_[3] = byte; }
        return count_ >= 4 && decode_packet(window_, out);
    }
private:
    uint8_t window_[4];
    uint8_t count_;
};

class Transport {
public:
    virtual ~Transport() {}
    virtual int read_byte() = 0;
    virtual bool write_bytes(const uint8_t* data, size_t len) = 0;
};

struct Telemetry {
    bool has_current, has_voltage, has_controller_temp, has_motor_temp, has_rudder, has_flags, has_eeprom_value;
    float current_a, voltage_v, controller_temp_c, motor_temp_c, rudder;
    bool rudder_valid;
    uint16_t flags;
    uint8_t eeprom_address, eeprom_value;

    Telemetry() { reset(); }
    void reset() {
        has_current = has_voltage = has_controller_temp = has_motor_temp = has_rudder = has_flags = has_eeprom_value = false;
        current_a = voltage_v = controller_temp_c = motor_temp_c = rudder = 0.0f;
        rudder_valid = false; flags = 0; eeprom_address = eeprom_value = 0;
    }
    bool apply(const Packet& p) {
        switch (p.code) {
        case CURRENT_CODE: current_a = decode_unsigned_centivalue(p.value); has_current = true; return true;
        case VOLTAGE_CODE: voltage_v = decode_unsigned_centivalue(p.value); has_voltage = true; return true;
        case CONTROLLER_TEMP_CODE: controller_temp_c = decode_signed_centivalue(p.value); has_controller_temp = true; return true;
        case MOTOR_TEMP_CODE: motor_temp_c = decode_signed_centivalue(p.value); has_motor_temp = true; return true;
        case RUDDER_SENSE_CODE: rudder_valid = decode_rudder(p.value, rudder); has_rudder = true; return true;
        case FLAGS_CODE: flags = p.value; has_flags = true; return true;
        case EEPROM_VALUE_CODE: eeprom_address = static_cast<uint8_t>(p.value & 0xffu); eeprom_value = static_cast<uint8_t>((p.value >> 8) & 0xffu); has_eeprom_value = true; return true;
        default: return false;
        }
    }
    bool synced() const { return (flags & SYNC_FLAG) != 0; }
    bool engaged() const { return (flags & ENGAGED_FLAG) != 0; }
    bool faulted() const { return is_fault_flag(flags); }
};

class Host {
public:
    explicit Host(Transport& transport) : transport_(transport) {}
    bool send_raw(uint8_t code, uint16_t value) { RawPacket p = encode_packet(code, value); return transport_.write_bytes(p.bytes, 4); }
    bool force_unsync() { const uint8_t bytes[4] = {0xff,0xff,0xff,0xff}; return transport_.write_bytes(bytes, 4); }
    bool send_command(float normalized) { return send_raw(COMMAND_CODE, encode_command(normalized)); }
    bool neutral() { return send_command(0.0f); }
    bool disengage() { return send_raw(DISENGAGE_CODE, 0); }
    bool reset_faults() { return send_raw(RESET_CODE, 0); }
    bool set_max_current_amps(float amps) { if (amps < 0.0f) amps = 0.0f; return send_raw(MAX_CURRENT_CODE, encode_centivalue(amps)); }
    bool set_max_controller_temp_c(float c) { if (c < 0.0f) c = 0.0f; return send_raw(MAX_CONTROLLER_TEMP_CODE, encode_centivalue(c)); }
    bool set_max_motor_temp_c(float c) { if (c < 0.0f) c = 0.0f; return send_raw(MAX_MOTOR_TEMP_CODE, encode_centivalue(c)); }
    bool set_rudder_range(float r) { return send_raw(RUDDER_RANGE_CODE, encode_rudder(r)); }
    bool set_rudder_min(float r) { return send_raw(RUDDER_MIN_CODE, encode_rudder(r)); }
    bool set_rudder_max(float r) { return send_raw(RUDDER_MAX_CODE, encode_rudder(r)); }
    bool set_max_slew(uint8_t fast, uint8_t slow) { return send_raw(MAX_SLEW_CODE, static_cast<uint16_t>(fast) | static_cast<uint16_t>(static_cast<uint16_t>(slow) << 8)); }
    bool set_clutch_pwm_and_brake(uint8_t pwm, bool brake) { return send_raw(CLUTCH_PWM_AND_BRAKE_CODE, static_cast<uint16_t>(pwm) | static_cast<uint16_t>((brake ? 1u : 0u) << 8)); }
    bool read_packet(Packet& out) { const int b = transport_.read_byte(); return b >= 0 && decoder_.push(static_cast<uint8_t>(b), out); }
    bool poll(Telemetry& t) { Packet p; return read_packet(p) && t.apply(p); }
private:
    Transport& transport_;
    Decoder decoder_;
};

class ControllerHandler {
public:
    virtual ~ControllerHandler() {}
    virtual void on_command(float normalized) = 0;
    virtual void on_disengage() = 0;
    virtual void on_reset_faults() = 0;
    virtual void on_max_current(float amps) = 0;
    virtual void on_max_controller_temp(float celsius) = 0;
    virtual void on_max_motor_temp(float celsius) = 0;
    virtual void on_rudder_range(float normalized) = 0;
    virtual void on_rudder_min(float normalized) = 0;
    virtual void on_rudder_max(float normalized) = 0;
    virtual void on_max_slew(uint8_t fast, uint8_t slow) = 0;
    virtual void on_clutch_pwm_and_brake(uint8_t pwm, bool brake) = 0;
    virtual void on_eeprom_read(uint8_t first, uint8_t last) = 0;
    virtual void on_eeprom_write(uint8_t address, uint8_t value) = 0;
    virtual void on_unknown_packet(uint8_t code, uint16_t value) { (void)code; (void)value; }
};

class Controller {
public:
    Controller(Transport& transport, ControllerHandler& handler) : transport_(transport), handler_(handler) {}
    bool poll() { const int b = transport_.read_byte(); if (b < 0) return false; Packet p; if (!decoder_.push(static_cast<uint8_t>(b), p)) return false; process(p); return true; }
    bool send_raw(uint8_t code, uint16_t value) { RawPacket p = encode_packet(code, value); return transport_.write_bytes(p.bytes, 4); }
    bool send_current(float amps) { return send_raw(CURRENT_CODE, encode_centivalue(amps)); }
    bool send_voltage(float volts) { return send_raw(VOLTAGE_CODE, encode_centivalue(volts)); }
    bool send_controller_temp(float c) { return send_raw(CONTROLLER_TEMP_CODE, encode_centivalue(c)); }
    bool send_motor_temp(float c) { return send_raw(MOTOR_TEMP_CODE, encode_centivalue(c)); }
    bool send_rudder(float r) { return send_raw(RUDDER_SENSE_CODE, encode_rudder(r)); }
    bool send_invalid_rudder() { return send_raw(RUDDER_SENSE_CODE, 65535u); }
    bool send_flags(uint16_t flags) { return send_raw(FLAGS_CODE, flags); }
    bool send_eeprom_value(uint8_t address, uint8_t value) { return send_raw(EEPROM_VALUE_CODE, static_cast<uint16_t>(address) | static_cast<uint16_t>(static_cast<uint16_t>(value) << 8)); }
private:
    void process(const Packet& p) {
        switch (p.code) {
        case COMMAND_CODE: if (p.value <= 2000u) handler_.on_command(decode_command_value(p.value)); else handler_.on_unknown_packet(p.code, p.value); break;
        case DISENGAGE_CODE: handler_.on_disengage(); break;
        case RESET_CODE: handler_.on_reset_faults(); break;
        case MAX_CURRENT_CODE: handler_.on_max_current(decode_unsigned_centivalue(p.value)); break;
        case MAX_CONTROLLER_TEMP_CODE: handler_.on_max_controller_temp(decode_signed_centivalue(p.value)); break;
        case MAX_MOTOR_TEMP_CODE: handler_.on_max_motor_temp(decode_signed_centivalue(p.value)); break;
        case RUDDER_RANGE_CODE: { float r; if (decode_rudder(p.value, r)) handler_.on_rudder_range(r); } break;
        case RUDDER_MIN_CODE: { float r; if (decode_rudder(p.value, r)) handler_.on_rudder_min(r); } break;
        case RUDDER_MAX_CODE: { float r; if (decode_rudder(p.value, r)) handler_.on_rudder_max(r); } break;
        case MAX_SLEW_CODE: handler_.on_max_slew(static_cast<uint8_t>(p.value & 0xffu), static_cast<uint8_t>((p.value >> 8) & 0xffu)); break;
        case CLUTCH_PWM_AND_BRAKE_CODE: handler_.on_clutch_pwm_and_brake(static_cast<uint8_t>(p.value & 0xffu), ((p.value >> 8) & 0xffu) != 0); break;
        case EEPROM_READ_CODE: handler_.on_eeprom_read(static_cast<uint8_t>(p.value & 0xffu), static_cast<uint8_t>((p.value >> 8) & 0xffu)); break;
        case EEPROM_WRITE_CODE: handler_.on_eeprom_write(static_cast<uint8_t>(p.value & 0xffu), static_cast<uint8_t>((p.value >> 8) & 0xffu)); break;
        default: handler_.on_unknown_packet(p.code, p.value); break;
        }
    }
    Transport& transport_;
    ControllerHandler& handler_;
    Decoder decoder_;
};

#ifdef ARDUINO
#include <Arduino.h>
class ArduinoStreamTransport : public Transport {
public:
    explicit ArduinoStreamTransport(Stream& stream) : stream_(stream) {}
    int read_byte() override { return stream_.available() ? stream_.read() : -1; }
    bool write_bytes(const uint8_t* data, size_t len) override { return stream_.write(data, len) == len; }
private:
    Stream& stream_;
};
#endif

} // namespace pypilot_servo_protocol

#if defined(PYPILOT_SERVO_PROTOCOL_ENABLE_LINUX_SERIAL) && !defined(ARDUINO)
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

namespace pypilot_servo_protocol {

class LinuxSerialTransport : public Transport {
public:
    LinuxSerialTransport() : fd_(-1) {}
    ~LinuxSerialTransport() override { close_device(); }
    bool open_device(const char* path, int baud = 38400) {
        close_device();
        fd_ = ::open(path, O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0) return false;
        termios tio;
        memset(&tio, 0, sizeof(tio));
        if (tcgetattr(fd_, &tio) != 0) { close_device(); return false; }
        cfmakeraw(&tio);
        tio.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
        tio.c_cflag &= static_cast<tcflag_t>(~PARENB);
        tio.c_cflag &= static_cast<tcflag_t>(~CSTOPB);
        tio.c_cflag &= static_cast<tcflag_t>(~CSIZE);
        tio.c_cflag |= CS8;
        tio.c_cc[VMIN] = 0;
        tio.c_cc[VTIME] = 1;
        const speed_t speed = baud == 115200 ? B115200 : baud == 57600 ? B57600 : baud == 19200 ? B19200 : baud == 9600 ? B9600 : B38400;
        cfsetispeed(&tio, speed);
        cfsetospeed(&tio, speed);
        if (tcsetattr(fd_, TCSANOW, &tio) != 0) { close_device(); return false; }
        tcflush(fd_, TCIOFLUSH);
        return true;
    }
    void close_device() { if (fd_ >= 0) { ::close(fd_); fd_ = -1; } }
    bool is_open() const { return fd_ >= 0; }
    int read_byte() override { uint8_t b = 0; return fd_ >= 0 && ::read(fd_, &b, 1) == 1 ? b : -1; }
    bool write_bytes(const uint8_t* data, size_t len) override {
        if (fd_ < 0) return false;
        size_t written = 0;
        while (written < len) {
            const ssize_t n = ::write(fd_, data + written, len - written);
            if (n < 0) { if (errno == EINTR || errno == EAGAIN) continue; return false; }
            written += static_cast<size_t>(n);
        }
        return true;
    }
private:
    int fd_;
};

} // namespace pypilot_servo_protocol
#endif
