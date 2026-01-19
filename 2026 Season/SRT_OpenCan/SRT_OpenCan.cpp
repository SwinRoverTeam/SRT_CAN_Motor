#include "SRT_OpenCan.h"

SRT_CanOpenMtr::SRT_CanOpenMtr(int (*sendfunc)(uint16_t, uint8_t, uint8_t*, bool), uint8_t nodeid, uint8_t gearratio) {
    _node_id = nodeid;
    can_send_msg = sendfunc;  // Fixed: no dereference needed
    _gear_ratio = gearratio;
}

int SRT_CanOpenMtr::process_msg(uint16_t can_id, uint8_t len, uint8_t* data) {
    if ((can_id & 0x7F) != _node_id) return -1; // Not our node
    // TODO: parse SDO/PDO responses later
    return 0;
}

int SRT_CanOpenMtr::send_sdo_write(uint16_t index, uint8_t sub, uint32_t value, uint8_t size) {
    uint16_t can_id = 0x600 + _node_id;
    uint8_t cs = (size == 1) ? 0x2F : (size == 2) ? 0x2B : 0x23;
    
    uint8_t data[8];
    data[0] = cs;
    data[1] = static_cast<uint8_t>(index & 0xFF);
    data[2] = static_cast<uint8_t>((index >> 8) & 0xFF);
    data[3] = sub;
    data[4] = static_cast<uint8_t>(value & 0xFF);
    data[5] = static_cast<uint8_t>((value >> 8) & 0xFF);
    data[6] = static_cast<uint8_t>((value >> 16) & 0xFF);
    data[7] = static_cast<uint8_t>((value >> 24) & 0xFF);
    
    return can_send_msg(can_id, 8, data, false);
}


int SRT_CanOpenMtr::enable_motor() {

    send_sdo_write(0x6040, 0x00, 6, 2);
    delay(10);
    send_sdo_write(0x6040, 0x00, 7, 2);
    delay(10);
    return send_sdo_write(0x6040, 0x00, 15, 2);
}

int SRT_CanOpenMtr::move_relative(int32_t steps, uint32_t velocity, uint32_t accel_ms, uint32_t decel_ms) {
    stop(); //Ensure there is not a movement already happening. Temporary, there are better ways of doing this!
    send_sdo_write(0x6060, 0x00, 1, 1);  // Position mode
    send_sdo_write(0x6081, 0x00, velocity*_gear_ratio, 4);// Profile velocity 1000
    send_sdo_write(0x6083, 0x00, accel_ms*_gear_ratio, 4);
    send_sdo_write(0x6084, 0x00, decel_ms*_gear_ratio, 4);
    enable_motor();  // Ensure enabled
    send_sdo_write(0x607A, 0x00, (uint32_t)steps*_gear_ratio, 4);  // Target position
    return send_sdo_write(0x6040, 0x00, 95, 2);  // Start relative move
}

int SRT_CanOpenMtr::move_absolute(int32_t steps,uint32_t velocity, uint32_t accel_ms, uint32_t decel_ms) {
    stop(); //Ensure there is not a movement already happening. Temporary, there are better ways of doing this!
    send_sdo_write(0x6060, 0x00, 1, 1);  // Position mode
    send_sdo_write(0x6081, 0x00, velocity*_gear_ratio, 4);// Profile velocity 1000
    send_sdo_write(0x6083, 0x00, accel_ms*_gear_ratio, 4);
    send_sdo_write(0x6084, 0x00, decel_ms*_gear_ratio, 4);
    enable_motor();  // Ensure enabled
    send_sdo_write(0x607A, 0x00, (uint32_t)steps*_gear_ratio, 4);  // Target position
    return send_sdo_write(0x6040, 0x00, 31, 2);
}

int SRT_CanOpenMtr::stop() {
    return send_sdo_write(0x6040, 0x00, 271, 2); // Stops the motor gently
}
int SRT_CanOpenMtr::Estop() {
    return send_sdo_write(0x6040, 0x00, 11, 2); // Stops the motor as quickly as possible
}
