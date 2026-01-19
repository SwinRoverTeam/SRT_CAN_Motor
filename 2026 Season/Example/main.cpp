#include <Arduino.h>
#include "driver/twai.h"
#include "SRT_Odrive.h"
#include "SRT_OpenCan.h"

// -------- CAN config --------
#define CAN_TX_GPIO GPIO_NUM_16
#define CAN_RX_GPIO GPIO_NUM_18
#define CAN_BAUDRATE_500K TWAI_TIMING_CONFIG_500KBITS()

// Fixed profile parameters
const uint32_t FIXED_VEL = 1000;   // 6081h BIGGER NUMBER = SLOWER SPEED
const uint32_t FIXED_ACC = 1000;    // 6083h
const uint32_t FIXED_DEC = 1000;    // 6084h 1 IS VERY FAST DO NOT GO BELOW 300 FOR ANY OF THESE!!!

// -------- Node ID arrays --------
// ODrive: 1+4n, between 1 and 29
//set odrive motors to as many as needed and and there can_id addresses
constexpr uint8_t ODRIVE_NODE_IDS[]   = {1, 5, 9, 13};
//do not touch this function this tests how many of each motor there m4 is of odrive
constexpr size_t  NUM_ODRIVE_MOTORS   = sizeof(ODRIVE_NODE_IDS) / sizeof(ODRIVE_NODE_IDS[0]);

// CANopen: 0..31, but 0 reserved for broadcast
//set lichuan motors to as many as needed and and there can_id addresses
constexpr uint8_t OPENCAN_NODE_IDS[]  = {2, 3, 4, 6};
//do not touch this function this tests how many of each motor there is of lichuan
constexpr size_t  NUM_OPENCAN_MOTORS  = sizeof(OPENCAN_NODE_IDS) / sizeof(OPENCAN_NODE_IDS[0]);

// -------- Shared CAN TX (don’t touch logic) --------
// if you want to read more on this go ahead https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/twai.html#_CPPv418twai_transmitPK18twai_message_t15TickType_t
int send_can_msg(uint16_t canid, uint8_t len, uint8_t* data, bool rtr){
    twai_message_t msg{};
    msg.identifier = canid;
    msg.extd = 0;// 11-bit ID normal frame length for can
    msg.rtr = rtr ? 1 : 0; // remote transmission request this is for requesting data
    msg.self = 0; // self reception request 
    msg.ss = 0; // single shot transmission

    //this makes sure that the length of data being sent is not more than 8 bytes if it is it trims it down to 8 bytes
    if (len > 8) len = 8; 
    msg.data_length_code = len;
    if (!rtr && len && data) memcpy(msg.data, data, len);
// pdMS_TO_TICKS converts milliseconds to RTOS ticks
    return (twai_transmit(&msg, pdMS_TO_TICKS(500)) == ESP_OK) ? 0 : -1;
}

// -------- TWAI init --------
bool init_twai(gpio_num_t tx, gpio_num_t rx) {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(tx, rx, TWAI_MODE_NORMAL);
    g_config.rx_queue_len = 20;
    twai_timing_config_t t_config = CAN_BAUDRATE_500K;
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        Serial.println("TWAI driver install failed");
        return false;
    }
    if (twai_start() != ESP_OK) {
        Serial.println("TWAI start failed");
        return false;
    }
    Serial.println("TWAI started @ 500 kbit/s");
    return true;
}

// -------- Motor objects (arrays) --------
SRT_OdriveMtr   odrives[NUM_ODRIVE_MOTORS] = {
    SRT_OdriveMtr(&send_can_msg, ODRIVE_NODE_IDS[0]), // First motor with can id 1
    SRT_OdriveMtr(&send_can_msg, ODRIVE_NODE_IDS[1]), // Second motor with can id 5
    SRT_OdriveMtr(&send_can_msg, ODRIVE_NODE_IDS[2]), // Third motor with can id 9
    SRT_OdriveMtr(&send_can_msg, ODRIVE_NODE_IDS[3]), // Fourth motor with can id 13
};

SRT_CanOpenMtr  opencans[NUM_OPENCAN_MOTORS] = {
    SRT_CanOpenMtr(&send_can_msg, OPENCAN_NODE_IDS[0]),
    SRT_CanOpenMtr(&send_can_msg, OPENCAN_NODE_IDS[1]),
    SRT_CanOpenMtr(&send_can_msg, OPENCAN_NODE_IDS[2]),
    SRT_CanOpenMtr(&send_can_msg, OPENCAN_NODE_IDS[3]),
};
void init_odrive_motors() {
    for (size_t i = 0; i < NUM_ODRIVE_MOTORS; ++i) {
        odrives[i].begin();
        //Populate cases when more motors are added or we want seperate settings for each motor
        switch (i) {
            default:
                odrives[i].set_cont_mode(position_control, trap_traj);
                odrives[i].set_axis_state(closed_loop_control);
                odrives[i].set_lim(10.0f, 20.0f);
                odrives[i].set_traj_vel_limit(5.0f);
                odrives[i].set_traj_accel_limits(5.0f, 5.0f);
                odrives[i].set_absolute_position(0.0f);
                break;
        }
    }
}

void init_opencan_motors() {
    for (size_t i = 0; i < NUM_OPENCAN_MOTORS; ++i) {
        opencans[i].enable_motor();
        delay(10);
    }
}

// -------- CAN RX: hand frames to all motors, they self-filter by node_id --------
void can_check_recv() {
    twai_message_t msg;
    while (twai_receive(&msg, pdMS_TO_TICKS(2)) == ESP_OK) {
        if (msg.rtr) continue;

        // ODrive motors
        for (size_t i = 0; i < NUM_ODRIVE_MOTORS; ++i) {
            odrives[i].process_msg(msg.identifier, msg.data_length_code, msg.data);
        }
        // CANopen motors
        for (size_t i = 0; i < NUM_OPENCAN_MOTORS; ++i) {
            opencans[i].process_msg(msg.identifier, msg.data_length_code, msg.data);
        }
    }
}


void setup() {
    Serial.begin(115200);
    delay(1000);

    if (!init_twai(CAN_TX_GPIO, CAN_RX_GPIO)) {
        Serial.println("CAN init failed");
        while (1) delay(1000);
    }

    init_odrive_motors();
    init_opencan_motors();
    
    // Debug: check axis state/error for motor 0
    Serial.print("Axis state: ");
    Serial.println(odrives[0].last_mtr_values.axis_state);
    Serial.print("Axis error: 0x");
    Serial.println(odrives[0].last_mtr_values.axis_error, HEX);

    Serial.println("All motors initialised");
}
void loop() {
    can_check_recv();// this checks for any incoming can messages and processes them accordingly. Dont remove stinky!


    // Check motors are in the correct state to use these commands

    //odrives[i].set_ip_pos(float ip_pos, int16_t vel, int16_t torque);
    //odrives[i].set_ip_vel(float vel, float torque);
    //odrives[i].set_absolute_position(float pos)

    //opencans[i].move_absolute(int32_t steps, uint32_t accel_ms, uint32_t decel_ms);
    //opencans[i].move_relative(int32_t steps, uint32_t accel_ms, uint32_t decel_ms);
    



    delay(5);
}


