#include "SRT_PiccoloCAN.h"

namespace PiccoloCAN {

// ---------------- helpers ----------------
static inline void put_i16_be(uint8_t *p, int16_t v) {
    p[0] = (uint8_t)((uint16_t)v >> 8);
    p[1] = (uint8_t)((uint16_t)v & 0xFF);
}

static inline int16_t clamp_cmd(int16_t v) {
    if (v < CMD_MIN) return CMD_MIN;
    if (v > CMD_MAX) return CMD_MAX;
    return v;
}

// ---------------- ID ----------------
uint32_t make_id(uint8_t msg_type, uint8_t address) {
    return ((uint32_t)(GROUP_ID & 0x1F) << 24) |
           ((uint32_t)msg_type << 16) |
           ((uint32_t)DEVICE_TYPE << 8) |
           (uint32_t)address;
}

void parse_id(uint32_t id, uint8_t &group, uint8_t &msg_type, uint8_t &dev_type, uint8_t &address) {
    group    = (id >> 24) & 0x1F;
    msg_type = (id >> 16) & 0xFF;
    dev_type = (id >> 8)  & 0xFF;
    address  = id & 0xFF;
}

// ---------------- Frame builders ----------------
Frame build_position(uint8_t address, int16_t command) {
    Frame f;
    f.id = make_id(PKT_POSITION_COMMAND, address);
    f.len = 2;
    put_i16_be(&f.data[0], clamp_cmd(command));
    return f;
}

Frame build_neutral(uint8_t address) {
    Frame f;
    f.id = make_id(PKT_NEUTRAL_COMMAND, address);
    f.len = 0;
    return f;
}

Frame build_enable(uint8_t address) {
    Frame f;
    f.id = make_id(PKT_ENABLE, address);
    f.len = 0;
    return f;
}

Frame build_disable(uint8_t address) {
    Frame f;
    f.id = make_id(PKT_DISABLE, address);
    f.len = 0;
    return f;
}

Frame build_request(uint8_t address, uint8_t msg_type) {
    Frame f;
    f.id = make_id(msg_type, address);
    f.len = 0;   // zero-length frame with same ID = poll
    return f;
}

Frame build_request_hf(uint8_t address, uint8_t packet_mask) {
    Frame f;
    f.id = make_id(PKT_SYSTEM_COMMAND, address);
    f.len = 2;
    f.data[0] = CMD_REQUEST_HF_DATA;
    f.data[1] = packet_mask;
    return f;
}

Frame build_multi_position(uint8_t group, int16_t a, int16_t b, int16_t c, int16_t d) {
    if (group < 1) group = 1;
    if (group > 16) group = 16;
    Frame f;
    f.id = make_id(PKT_MULTI_COMMAND_1 + (group - 1), BROADCAST);  // must be broadcast
    f.len = 8;
    put_i16_be(&f.data[0], clamp_cmd(a));
    put_i16_be(&f.data[2], clamp_cmd(b));
    put_i16_be(&f.data[4], clamp_cmd(c));
    put_i16_be(&f.data[6], clamp_cmd(d));
    return f;
}

} // namespace PiccoloCAN

// =====================================================================
// SRT_PiccoloServo
// =====================================================================
using namespace PiccoloCAN;

SRT_PiccoloServo::SRT_PiccoloServo(int (*sendfunc)(uint32_t, uint8_t, uint8_t*), uint8_t nodeid) {
    _send = sendfunc;
    _node_id = nodeid;
}

int SRT_PiccoloServo::send_frame(const Frame &f) {
    uint8_t buf[8];
    memcpy(buf, f.data, 8);
    return _send(f.id, f.len, buf);
}

int SRT_PiccoloServo::enable()       { return send_frame(build_enable(_node_id)); }
int SRT_PiccoloServo::disable()      { return send_frame(build_disable(_node_id)); }
int SRT_PiccoloServo::go_neutral()   { return send_frame(build_neutral(_node_id)); }
int SRT_PiccoloServo::set_position(int16_t c) { return send_frame(build_position(_node_id, c)); }

int SRT_PiccoloServo::request_status_a() { return send_frame(build_request(_node_id, PKT_STATUS_A)); }
int SRT_PiccoloServo::request_status_b() { return send_frame(build_request(_node_id, PKT_STATUS_B)); }
int SRT_PiccoloServo::request_status_c() { return send_frame(build_request(_node_id, PKT_STATUS_C)); }
int SRT_PiccoloServo::request_hf(uint8_t mask) { return send_frame(build_request_hf(_node_id, mask)); }

int SRT_PiccoloServo::process_msg(uint32_t can_id, uint8_t len, const uint8_t *d) {
    uint8_t group, msg, dev, addr;
    parse_id(can_id, group, msg, dev, addr);

    if (group != GROUP_ID || dev != DEVICE_TYPE || addr != _node_id) return -1;

    switch (msg) {
        case PKT_STATUS_A:
            if (len < 8) return -1;
            _a.enabled      = d[0] & 0x80;
            _a.mode         = (d[0] >> 4) & 0x07;
            _a.cmd_received = d[0] & 0x08;
            _a.warnings     = d[1];
            _a.errors       = d[3];
            _a.position     = (int16_t)((d[4] << 8) | d[5]);
            _a.command      = (int16_t)((d[6] << 8) | d[7]);
            _a.stamp_ms     = millis();
            _a.valid        = true;
            return 0;

        case PKT_STATUS_B:
            if (len < 5) return -1;   // duty (byte 5) and speed (6..7) optional
            _b.current_mA    = ((uint16_t)((d[0] << 8) | d[1])) * 10;
            _b.voltage_mV    = ((uint16_t)((d[2] << 8) | d[3])) * 10;
            _b.temperature_C = (int8_t)d[4];
            _b.duty_pct      = (len >= 6) ? (int8_t)d[5] : 0;
            _b.speed_dps     = (len >= 8) ? (int16_t)((d[6] << 8) | d[7]) : 0;
            _b.stamp_ms      = millis();
            _b.valid         = true;
            return 0;

        case PKT_STATUS_C:
            if (len < 2) return -1;
            _pos_c = (int16_t)((d[0] << 8) | d[1]);
            return 0;

        default:
            return -1;
    }
}