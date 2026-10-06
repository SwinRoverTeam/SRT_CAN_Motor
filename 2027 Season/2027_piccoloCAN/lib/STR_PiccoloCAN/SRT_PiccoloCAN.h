#ifndef SRT_PICCOLOCAN_H
#define SRT_PICCOLOCAN_H

#include "Arduino.h"

// Currawong CE985 / CBS-15 CAN servo - PiccoloCAN protocol (ICD rev 2.14)
// CAN 2.0B extended (29-bit) ID, 1 Mbit/s, data is BIG endian.
//
// 29-bit ID layout:
//   [28..24] Group ID      = 0x07 (servo)
//   [23..16] Message Type  = packet identifier (e.g. 0x10 position cmd)
//   [15..8]  Device Type   = 0x00 (servo)
//   [7..0]   Device Address= servo node ID (0..254), 0xFF = broadcast

namespace PiccoloCAN {

constexpr uint8_t GROUP_ID    = 0x07;
constexpr uint8_t DEVICE_TYPE = 0x00;
constexpr uint8_t BROADCAST   = 0xFF;

// Packet identifiers (message type field)
enum PacketId : uint8_t {
    PKT_MULTI_COMMAND_1 = 0x00,   // 0x00..0x0F: servos 1-4, 5-8, ... 61-64
    PKT_POSITION_COMMAND = 0x10,
    PKT_NEUTRAL_COMMAND  = 0x15,
    PKT_DISABLE          = 0x20,
    PKT_ENABLE           = 0x21,
    PKT_SYSTEM_COMMAND   = 0x50,
    PKT_SET_TITLE        = 0x51,
    PKT_STATUS_A         = 0x60,
    PKT_STATUS_B         = 0x61,
    PKT_STATUS_C         = 0x62,
    PKT_ACCELEROMETER    = 0x64,
    PKT_ADDRESS          = 0x70,
    PKT_TITLE            = 0x71,
    PKT_FIRMWARE         = 0x72,
    PKT_SYSTEM_INFO      = 0x73,
    PKT_TELEMETRY_CONFIG = 0x74,
    PKT_SETTINGS_INFO    = 0x75
};

// First byte of a PKT_SYSTEM_COMMAND frame
enum SysCmd : uint8_t {
    CMD_REQUEST_HF_DATA = 67
};

// High-frequency telemetry selection bits (byte 1 of REQUEST_HF_DATA)
constexpr uint8_t HF_STATUS_A = 1 << 7;
constexpr uint8_t HF_STATUS_B = 1 << 6;
constexpr uint8_t HF_STATUS_C = 1 << 5;
constexpr uint8_t HF_ACCEL    = 1 << 3;

constexpr int16_t CMD_MIN = -20000;
constexpr int16_t CMD_MAX = 20000;

// A raw CAN frame (extended ID)
struct Frame {
    uint32_t id  = 0;
    uint8_t  len = 0;
    uint8_t  data[8] = {0};
};

// ---------------- Frame builder ----------------
uint32_t make_id(uint8_t msg_type, uint8_t address);
void     parse_id(uint32_t id, uint8_t &group, uint8_t &msg_type, uint8_t &dev_type, uint8_t &address);

Frame build_position(uint8_t address, int16_t command);
Frame build_neutral(uint8_t address);
Frame build_enable(uint8_t address);
Frame build_disable(uint8_t address);
Frame build_request(uint8_t address, uint8_t msg_type);                  // zero-length poll
Frame build_request_hf(uint8_t address, uint8_t packet_mask);            // 500 Hz for 1 s
// Command 4 consecutive servos in one broadcast. group = 1..16 (servos 1-4 = group 1)
Frame build_multi_position(uint8_t group, int16_t a, int16_t b, int16_t c, int16_t d);

} // namespace PiccoloCAN

// ---------------- Telemetry structs ----------------
struct PiccoloStatusA {
    bool     enabled = false;
    uint8_t  mode = 0;
    bool     cmd_received = false;
    uint8_t  warnings = 0;     // byte 1: b7 overCurrent, b6 overTemp, b5 overAccel, b4 invalidInput,
                               //         b3 position, b2 potCal, b1 underVolt, b0 overVolt
    uint8_t  errors = 0;       // byte 3: b7 position, b6 pot, b5 acc, b4 mapCal, b3 settings, b2 health, b1 tracking
    int16_t  position = 0;
    int16_t  command = 0;
    uint32_t stamp_ms = 0;
    bool     valid = false;
};

struct PiccoloStatusB {
    uint16_t current_mA = 0;
    uint16_t voltage_mV = 0;
    int8_t   temperature_C = 0;
    int8_t   duty_pct = 0;
    int16_t  speed_dps = 0;
    uint32_t stamp_ms = 0;
    bool     valid = false;
};

// ---------------- Servo class ----------------
class SRT_PiccoloServo {
public:
    // sendfunc(id29, len, data) -> 0 on success. Must send an EXTENDED frame.
    SRT_PiccoloServo(int (*sendfunc)(uint32_t, uint8_t, uint8_t*), uint8_t nodeid);

    // Commands (not acknowledged by the servo)
    int enable();
    int disable();
    int set_position(int16_t command);   // clamped to +/-20000
    int go_neutral();

    // Telemetry polling (non-blocking - reply arrives via process_msg)
    int request_status_a();
    int request_status_b();
    int request_status_c();
    int request_hf(uint8_t mask);

    // Feed every received extended frame in here. Returns 0 if consumed, -1 otherwise.
    int process_msg(uint32_t can_id, uint8_t len, const uint8_t* data);

    const PiccoloStatusA &status_a() const { return _a; }
    const PiccoloStatusB &status_b() const { return _b; }
    int16_t position() const { return _pos_c; }
    bool is_enabled() const { return _a.valid && _a.enabled; }
    bool has_fault() const { return _a.valid && _a.errors != 0; }

    uint8_t node_id() const { return _node_id; }

private:
    uint8_t _node_id;
    int (*_send)(uint32_t, uint8_t, uint8_t*);
    PiccoloStatusA _a;
    PiccoloStatusB _b;
    int16_t _pos_c = 0;

    int send_frame(const PiccoloCAN::Frame &f);
};

#endif