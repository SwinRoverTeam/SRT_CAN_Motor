#ifndef ODRIVECAN_H  // Add _H suffix, proper spacing
#define ODRIVECAN_H
#include "Arduino.h"  // Add quotes for local include


union float_conv {
    float    flt;
    uint32_t u32;
};

enum cmd_id {
    get_version              = 0x000,
    heartbeat                = 0x001,
    estop                    = 0x002,
    get_error                = 0x003,
    rx_sdo                   = 0x004,
    tx_sdo                   = 0x005,
    address                  = 0x006,
    set_axis_state           = 0x007,
    get_encoder_estimates    = 0x009,
    set_controller_mode      = 0x00b,
    set_input_pos            = 0x00c,
    set_input_vel            = 0x00d,
    set_input_torque         = 0x00e,
    set_limit                = 0x00f,
    set_traj_vel_limits      = 0x011,
    set_traj_accel_limits    = 0x012,
    set_traj_inertia         = 0x013,
    get_iq                   = 0x014,
    get_temp                 = 0x015,
    reboot                   = 0x016,
    get_bus_voltage_current  = 0x017,
    clear_errors             = 0x018,
    set_abs_position         = 0x019,
    set_pos_gain             = 0x01a,
    set_vel_gains            = 0x01b,
    get_torques              = 0x01c,
    get_powers               = 0x01d,
    enter_dfu_mode           = 0x01f
};

enum proccess_return {
    not_this_node        = -0x1,
    success              = 0x00,
    can_bus_fault        = 0x01,
    can_bus_no_response  = 0x02,
    can_transmission_fail= 0x03,
    not_info_cmd         = 0x04,
    not_valid_cmd        = 0x05,
    invalid_data_length  = 0x06,
    input_out_of_range   = 0x07
};

enum axis_states {
    undef                           = 0x0,
    idle                            = 0x1,
    startup_sequence                = 0x2,
    full_calibration_sequence       = 0x3,
    motor_calibration               = 0x4,
    sensorless_control              = 0x5,
    encoder_index_search            = 0x6,
    encoder_offset_calibration      = 0x7,
    closed_loop_control             = 0x8,
    lockin_spin                     = 0x9,
    encoder_dir_find                = 0xA,
    homing                          = 0xB,
    encoder_hall_polarity_calibration = 0xC,
    encoder_hall_phase_calibration  = 0xD,
    anticogging_calibration         = 0xE
};

enum control_mode {
    voltage_control = 0x0,
    torque_control  = 0x1,
    velocity_control= 0x2,
    position_control= 0x3
};

enum input_mode {
    inactive    = 0x0,
    passthrough = 0x1,
    vel_ramp    = 0x2,
    pos_filter  = 0x3,
    mix_channels= 0x4,
    trap_traj   = 0x5,
    torque_ramp = 0x6,
    mirror      = 0x7,
    tuning      = 0x8
};

enum action {
    a_reboot            = 0x0,
    a_save_configuration= 0x1,
    a_erase_configuration=0x2,
    a_enter_dfu_mode    = 0x3
};

struct errors {
    bool initalising;
    bool system_level;
    bool timing_error;
    bool missing_estimate;
    bool bad_config;
    bool drv_fault;
    bool missing_input;
    bool dc_bus_over_voltage;
    bool dc_bus_under_voltage;
    bool dc_bus_over_current;
    bool dc_bus_over_regen_current;
    bool current_limit_violation;
    bool motor_over_temp;
    bool inverter_over_temp;
    bool velocity_limit_violation;
    bool position_limit_violation;
    bool watchdog_timer_expired;
    bool estop_requested;
    bool spinout_detected;
    bool brake_resistor_disarmed;
    bool thermistor_disconnected;
    bool calibration_error;
};

struct mtr_values {
    uint32_t axis_error;
    uint8_t  axis_state;
    uint32_t active_errors;
    uint32_t disarm_reason;
    float    pos_estimate;
    float    vel_estimate;
    float    iq_setpoint;
    float    iq_measured;
    float    fet_temp;
    float    motor_temp;
    float    bus_voltage;
    float    bus_current;
    float    torque_tar;
    float    torque_estimate;
    float    elec_power;
    float    mech_power;
    errors   current_errors;
};

class SRT_OdriveMtr {
private:
    uint8_t  _node_id; // Be careful 0x3f is the broadcast ID!
    uint8_t _gear_ratio;
    int (*can_send_msg)(uint16_t can_id, uint8_t len, uint8_t* data, bool rtr);
    uint32_t   _mtr_last_hb;
    float_conv flt_cnv;
    uint16_t   _timeout;

    void proccess_errors(uint32_t input);

public:
    SRT_OdriveMtr(int (*send_func)(uint16_t can_id, uint8_t len, uint8_t* data, bool rtr),
                  uint8_t node_id, uint8_t gear_ratio);

    void begin();
    int  process_cmd(cmd_id cmd, uint8_t len, uint8_t* data);
    int  process_msg(uint16_t can_id, uint8_t len, uint8_t* data);
    int  stop();
    bool config_motor();
    int  req_info_cmd(cmd_id cmd);
    bool set_timeout(uint16_t timeout_ms);

    int set_axis_state(uint32_t state);
    int set_cont_mode(uint32_t cont_mode, uint32_t ip_mode);
    int set_ip_pos(float ip_pos, int16_t vel, int16_t torque);
    int set_ip_vel(float vel, float torque);
    int set_ip_torq(float torque);
    int set_lim(float vel_lim, float cur_lim);
    int set_traj_vel_limit(float vel_lim);
    int set_traj_accel_limits(float accel_limit, float decel_limit);
    int set_traj_inertia(float interia);
    int reboot_mtr(uint8_t action);
    int clear_errors();
    int set_absolute_position(float pos);
    int set_position_gain(float pos_gain);
    int set_velocity_gains(float vel_gain, float vel_integ_gain);

    ~SRT_OdriveMtr();

    uint8_t node_id();
    bool    mtr_connected();
    mtr_values last_mtr_values;
};

#endif