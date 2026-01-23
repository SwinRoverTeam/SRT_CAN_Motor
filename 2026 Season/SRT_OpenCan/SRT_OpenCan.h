#ifndef SRT_CANOPEN_H
#define SRT_CANOPEN_H

#include "Arduino.h"
#include "driver/twai.h"

class SRT_CanOpenMtr {
private:
    uint8_t _node_id;
    uint8_t _gear_ratio;
    int (*can_send_msg)(uint16_t, uint8_t, uint8_t*, bool);
    int send_sdo_write(uint16_t index, uint8_t sub, uint32_t value, uint8_t size);
    int send_sdo_read(uint16_t index, uint8_t sub, uint8_t size);
public:
    SRT_CanOpenMtr(int (*sendfunc)(uint16_t, uint8_t, uint8_t*, bool), uint8_t nodeid, uint8_t gearratio);
    int process_msg(uint16_t can_id, uint8_t len, uint8_t* data);
    int enable_motor();
    int move_relative(int32_t steps,uint32_t velocity, uint32_t accel_ms = 1000, uint32_t decel_ms = 1000);
    int move_absolute(int32_t target,uint32_t velocity, uint32_t accel_ms = 1000, uint32_t decel_ms = 1000);
    bool is_motor_running();

    int stop();
    int Estop();

    //=========== Homing attempts =========
    int do_homing(); // run homing sequence, set current pos = 0
};

#endif
