#include "SRT_Odrive.h"

SRT_OdriveMtr::SRT_OdriveMtr(
    int (*send_func)(uint16_t can_id, uint8_t len, uint8_t* data, bool rtr),
    uint8_t node_id,
    uint8_t gear_ratio
) {
    _node_id     = node_id;
    _gear_ratio = gear_ratio;
    can_send_msg = send_func;
    _timeout     = 5000;
}

void SRT_OdriveMtr::begin() {
    if (_node_id == 0x3f) {
        Serial.println("[SRT_Odrive - Warning] 0x3f is the broadcast node ID!");
    }
}

/*
    Use when the main code is reading the can messages and passing them to the correct motor

    All can messages use intel (little-endian), i.e. if bytes 0-3 are the data for an unsigned 32 bit integer,
    byte 3 is the most significant byte and 1 is the least significant byte.
*/
int SRT_OdriveMtr::process_cmd(cmd_id cmd, uint8_t len, uint8_t* data) {
    int error_state = success;

    switch (cmd) {
        case get_version:
            //Bin, I don't care
            break;

        case heartbeat:
            _mtr_last_hb = millis();
            if (len < 5) {
                //Err
                return invalid_data_length;
            }
            last_mtr_values.axis_error =
                (data[3] << 24) + (data[2] << 16) + (data[1] << 8) + (data[0]);
            last_mtr_values.axis_state = data[4];
            //Dont really care about the rest.
            break;

        case get_error:
            if (len < 8) {
                //err
                return invalid_data_length;
            }
            last_mtr_values.active_errors =
                (data[3] << 24) + (data[2] << 16) + (data[1] << 8) + (data[0]);
            last_mtr_values.disarm_reason =
                (data[7] << 24) + (data[6] << 16) + (data[5] << 8) + (data[4]);
            proccess_errors(last_mtr_values.active_errors);
            break;

        case tx_sdo:
            //Dont care
            break;

        case address:
            //Dont care
            break;

        case get_encoder_estimates:
            if (len < 8) {
                //err
                return invalid_data_length;
            }
            flt_cnv.u32 =
                (data[3] << 24) + (data[2] << 16) + (data[1] << 8) + (data[0]);
            last_mtr_values.pos_estimate = flt_cnv.flt;

            flt_cnv.u32 =
                (data[7] << 24) + (data[6] << 16) + (data[5] << 8) + (data[4]);
            last_mtr_values.vel_estimate = flt_cnv.flt;
            break;

        case get_iq:
            if (len < 8) {
                //err
                return invalid_data_length;
            }
            flt_cnv.u32 =
                (data[3] << 24) + (data[2] << 16) + (data[1] << 8) + (data[0]);
            last_mtr_values.iq_setpoint = flt_cnv.flt;

            flt_cnv.u32 =
                (data[7] << 24) + (data[6] << 16) + (data[5] << 8) + (data[4]);
            last_mtr_values.iq_measured = flt_cnv.flt;
            break;

        case get_temp:
            if (len < 8) {
                //err
                return invalid_data_length;
            }
            flt_cnv.u32 =
                (data[3] << 24) + (data[2] << 16) + (data[1] << 8) + (data[0]);
            last_mtr_values.fet_temp = flt_cnv.flt;

            flt_cnv.u32 =
                (data[7] << 24) + (data[6] << 16) + (data[5] << 8) + (data[4]);
            last_mtr_values.motor_temp = flt_cnv.flt;
            break;

        case get_bus_voltage_current:
            if (len < 8) {
                //err
                return invalid_data_length;
            }
            flt_cnv.u32 =
                (data[3] << 24) + (data[2] << 16) + (data[1] << 8) + (data[0]);
            last_mtr_values.bus_voltage = flt_cnv.flt;

            flt_cnv.u32 =
                (data[7] << 24) + (data[6] << 16) + (data[5] << 8) + (data[4]);
            last_mtr_values.bus_current = flt_cnv.flt;
            break;

        case get_torques:
            if (len < 8) {
                //err
                return invalid_data_length;
            }
            flt_cnv.u32 =
                (data[3] << 24) + (data[2] << 16) + (data[1] << 8) + (data[0]);
            last_mtr_values.torque_tar = flt_cnv.flt;

            flt_cnv.u32 =
                (data[7] << 24) + (data[6] << 16) + (data[5] << 8) + (data[4]);
            last_mtr_values.torque_estimate = flt_cnv.flt;
            break;

        case get_powers:
            if (len < 8) {
                //err
                return invalid_data_length;
            }
            flt_cnv.u32 =
                (data[3] << 24) + (data[2] << 16) + (data[1] << 8) + (data[0]);
            last_mtr_values.elec_power = flt_cnv.flt;

            flt_cnv.u32 =
                (data[7] << 24) + (data[6] << 16) + (data[5] << 8) + (data[4]);
            last_mtr_values.mech_power = flt_cnv.flt;
            break;

        default:
            error_state = not_valid_cmd;
            break;
    }

    return error_state;
}

/*
    Use when only the motors on the bus
    Pass the full can id into the passing
*/
int SRT_OdriveMtr::process_msg(uint16_t can_id, uint8_t len, uint8_t* data) {
    // Left most 6 bits are the node id
    if ((can_id >> 5) != _node_id) {
        return not_this_node;
    }

    // Rest (5 bits) are the command ID
    uint8_t cmd = (can_id & 0x1F);
    return process_cmd((cmd_id)cmd, len, data);
}

bool SRT_OdriveMtr::set_timeout(uint16_t timeout_ms) {
    _timeout = timeout_ms;
    return true;
}

int SRT_OdriveMtr::req_info_cmd(cmd_id cmd) {
    switch (cmd) {
        case get_version:
        case heartbeat:
        case get_error:
        case tx_sdo:
        case address:
        case get_encoder_estimates:
        case get_iq:
        case get_temp:
        case get_bus_voltage_current:
        case get_torques:
        case get_powers:
            return can_send_msg(
                (_node_id << 5) + ((uint8_t)cmd),
                0,
                0,
                true
            );
        default:
            Serial.println("[SRT_Odrive] Not an info cmd");
            return not_info_cmd;
    }
}

/*
    Set motor values
*/
int SRT_OdriveMtr::stop() {
    return can_send_msg(
        (_node_id << 5) + ((uint8_t)cmd_id::estop),
        0,
        0,
        false
    );
}

bool SRT_OdriveMtr::config_motor() {
    // TODO: add your one-shot config sequence if needed
    return false; // explicit stub
}

int SRT_OdriveMtr::set_axis_state(uint32_t state) {
    if (state > anticogging_calibration) {
        return input_out_of_range;
    }

    uint8_t data[4];
    data[3] = (state >> 24) & 0xFF;
    data[2] = (state >> 16) & 0xFF;
    data[1] = (state >> 8)  & 0xFF;
    data[0] =  state        & 0xFF;

    return can_send_msg(
        (_node_id << 5) + ((uint8_t)cmd_id::set_axis_state),
        4,
        data,
        false
    );
}

int SRT_OdriveMtr::set_cont_mode(uint32_t cont_mode, uint32_t ip_mode) {
    if ((cont_mode > position_control) || (ip_mode > tuning)) {
        return input_out_of_range;
    }

    uint8_t data[8];
    data[7] = (ip_mode >> 24) & 0xFF;
    data[6] = (ip_mode >> 16) & 0xFF;
    data[5] = (ip_mode >> 8)  & 0xFF;
    data[4] =  ip_mode        & 0xFF;

    data[3] = (cont_mode >> 24) & 0xFF;
    data[2] = (cont_mode >> 16) & 0xFF;
    data[1] = (cont_mode >> 8)  & 0xFF;
    data[0] =  cont_mode        & 0xFF;

    return can_send_msg(
        (_node_id << 5) + ((uint8_t)cmd_id::set_controller_mode),
        8,
        data,
        false
    );
}

int SRT_OdriveMtr::set_ip_pos(float ip_pos, int16_t vel, int16_t torque) {
    uint8_t data[8];

    flt_cnv.flt = ip_pos*_gear_ratio;
    data[7] = (torque >> 8) & 0xFF;
    data[6] =  torque       & 0xFF;
    data[5] = (vel    >> 8) & 0xFF;
    data[4] =  vel          & 0xFF;
    data[3] = (flt_cnv.u32 >> 24) & 0xFF;
    data[2] = (flt_cnv.u32 >> 16) & 0xFF;
    data[1] = (flt_cnv.u32 >> 8)  & 0xFF;
    data[0] =  flt_cnv.u32        & 0xFF;

    return can_send_msg(
        (_node_id << 5) + ((uint8_t)cmd_id::set_input_pos),
        8,
        data,
        false
    );
}

int SRT_OdriveMtr::set_ip_vel(float vel, float torque) {
    uint8_t data[8];

    flt_cnv.flt = torque/_gear_ratio;
    data[7] = (flt_cnv.u32 >> 24) & 0xFF;
    data[6] = (flt_cnv.u32 >> 16) & 0xFF;
    data[5] = (flt_cnv.u32 >> 8)  & 0xFF;
    data[4] =  flt_cnv.u32        & 0xFF;

    flt_cnv.flt = vel*_gear_ratio;
    data[3] = (flt_cnv.u32 >> 24) & 0xFF;
    data[2] = (flt_cnv.u32 >> 16) & 0xFF;
    data[1] = (flt_cnv.u32 >> 8)  & 0xFF;
    data[0] =  flt_cnv.u32        & 0xFF;

    return can_send_msg(
        (_node_id << 5) + ((uint8_t)cmd_id::set_input_vel),
        8,
        data,
        false
    );
}

int SRT_OdriveMtr::set_ip_torq(float torque) {
    uint8_t data[4];

    flt_cnv.flt = torque/_gear_ratio;
    data[3] = (flt_cnv.u32 >> 24) & 0xFF;
    data[2] = (flt_cnv.u32 >> 16) & 0xFF;
    data[1] = (flt_cnv.u32 >> 8)  & 0xFF;
    data[0] =  flt_cnv.u32        & 0xFF;

    return can_send_msg(
        (_node_id << 5) + ((uint8_t)cmd_id::set_input_torque),
        4,
        data,
        false
    );
}

int SRT_OdriveMtr::set_lim(float vel_lim, float cur_lim) {
    uint8_t data[8];

    flt_cnv.flt = cur_lim;
    data[7] = (flt_cnv.u32 >> 24) & 0xFF;
    data[6] = (flt_cnv.u32 >> 16) & 0xFF;
    data[5] = (flt_cnv.u32 >> 8)  & 0xFF;
    data[4] =  flt_cnv.u32        & 0xFF;

    flt_cnv.flt = vel_lim*_gear_ratio;
    data[3] = (flt_cnv.u32 >> 24) & 0xFF;
    data[2] = (flt_cnv.u32 >> 16) & 0xFF;
    data[1] = (flt_cnv.u32 >> 8)  & 0xFF;
    data[0] =  flt_cnv.u32        & 0xFF;

    return can_send_msg(
        (_node_id << 5) + ((uint8_t)cmd_id::set_limit),
        8,
        data,
        false
    );
}

int SRT_OdriveMtr::set_traj_vel_limit(float vel_lim) {
    uint8_t data[4];

    flt_cnv.flt = vel_lim*_gear_ratio;
    data[3] = (flt_cnv.u32 >> 24) & 0xFF;
    data[2] = (flt_cnv.u32 >> 16) & 0xFF;
    data[1] = (flt_cnv.u32 >> 8)  & 0xFF;
    data[0] =  flt_cnv.u32        & 0xFF;

    return can_send_msg(
        (_node_id << 5) + ((uint8_t)cmd_id::set_traj_vel_limits),
        4,
        data,
        false
    );
}

int SRT_OdriveMtr::set_traj_accel_limits(float accel_limit, float decel_limit) {
    uint8_t data[8];

    flt_cnv.flt = decel_limit*_gear_ratio;
    data[7] = (flt_cnv.u32 >> 24) & 0xFF;
    data[6] = (flt_cnv.u32 >> 16) & 0xFF;
    data[5] = (flt_cnv.u32 >> 8)  & 0xFF;
    data[4] =  flt_cnv.u32        & 0xFF;

    flt_cnv.flt = accel_limit*_gear_ratio;
    data[3] = (flt_cnv.u32 >> 24) & 0xFF;
    data[2] = (flt_cnv.u32 >> 16) & 0xFF;
    data[1] = (flt_cnv.u32 >> 8)  & 0xFF;
    data[0] =  flt_cnv.u32        & 0xFF;

    return can_send_msg(
        (_node_id << 5) + ((uint8_t)cmd_id::set_traj_accel_limits),
        8,
        data,
        false
    );
}

int SRT_OdriveMtr::set_traj_inertia(float interia) {
    uint8_t data[4];

    flt_cnv.flt = interia;
    data[3] = (flt_cnv.u32 >> 24) & 0xFF;
    data[2] = (flt_cnv.u32 >> 16) & 0xFF;
    data[1] = (flt_cnv.u32 >> 8)  & 0xFF;
    data[0] =  flt_cnv.u32        & 0xFF;

    return can_send_msg(
        (_node_id << 5) + ((uint8_t)cmd_id::set_traj_inertia),
        4,
        data,
        false
    );
}

int SRT_OdriveMtr::reboot_mtr(uint8_t action) {
    if (action > a_enter_dfu_mode) {
        return input_out_of_range;
    }

    uint8_t data[1];
    data[0] = action;

    return can_send_msg(
        (_node_id << 5) + ((uint8_t)cmd_id::reboot),
        1,
        data,
        false
    );
}

int SRT_OdriveMtr::clear_errors() {
    // ODrive expects a zero-length data frame for clear_errors.
    return can_send_msg(
        (_node_id << 5) + ((uint8_t)cmd_id::clear_errors),
        0,
        0,
        false
    );
}

int SRT_OdriveMtr::set_absolute_position(float pos) {
    uint8_t data[4];

    flt_cnv.flt = pos*_gear_ratio;
    data[3] = (flt_cnv.u32 >> 24) & 0xFF;
    data[2] = (flt_cnv.u32 >> 16) & 0xFF;
    data[1] = (flt_cnv.u32 >> 8)  & 0xFF;
    data[0] =  flt_cnv.u32        & 0xFF;

    return can_send_msg(
        (_node_id << 5) + ((uint8_t)cmd_id::set_abs_position),
        4,
        data,
        false
    );
}

int SRT_OdriveMtr::set_position_gain(float pos_gain) {
    uint8_t data[4];

    flt_cnv.flt = pos_gain;
    data[3] = (flt_cnv.u32 >> 24) & 0xFF;
    data[2] = (flt_cnv.u32 >> 16) & 0xFF;
    data[1] = (flt_cnv.u32 >> 8)  & 0xFF;
    data[0] =  flt_cnv.u32        & 0xFF;

    return can_send_msg(
        (_node_id << 5) + ((uint8_t)cmd_id::set_pos_gain),
        4,
        data,
        false
    );
}

int SRT_OdriveMtr::set_velocity_gains(float vel_gain, float vel_integ_gain) {
    uint8_t data[8];

    flt_cnv.flt = vel_integ_gain;
    data[7] = (flt_cnv.u32 >> 24) & 0xFF;
    data[6] = (flt_cnv.u32 >> 16) & 0xFF;
    data[5] = (flt_cnv.u32 >> 8)  & 0xFF;
    data[4] =  flt_cnv.u32        & 0xFF;

    flt_cnv.flt = vel_gain;
    data[3] = (flt_cnv.u32 >> 24) & 0xFF;
    data[2] = (flt_cnv.u32 >> 16) & 0xFF;
    data[1] = (flt_cnv.u32 >> 8)  & 0xFF;
    data[0] =  flt_cnv.u32        & 0xFF;

    return can_send_msg(
        (_node_id << 5) + ((uint8_t)cmd_id::set_vel_gains),
        8,
        data,
        false
    );
}

uint8_t SRT_OdriveMtr::node_id() {
    return _node_id;
}

bool SRT_OdriveMtr::mtr_connected() {
    return (millis() - _mtr_last_hb < _timeout);
}

void SRT_OdriveMtr::proccess_errors(uint32_t input) {
    last_mtr_values.current_errors.initalising              = (input >> 0)  & 0x1;
    last_mtr_values.current_errors.system_level             = (input >> 1)  & 0x1;
    last_mtr_values.current_errors.timing_error             = (input >> 2)  & 0x1;
    last_mtr_values.current_errors.missing_estimate         = (input >> 3)  & 0x1;
    last_mtr_values.current_errors.bad_config               = (input >> 4)  & 0x1;
    last_mtr_values.current_errors.drv_fault                = (input >> 5)  & 0x1;
    last_mtr_values.current_errors.missing_input            = (input >> 6)  & 0x1;
    last_mtr_values.current_errors.dc_bus_over_voltage      = (input >> 7)  & 0x1;
    last_mtr_values.current_errors.dc_bus_under_voltage     = (input >> 8)  & 0x1;
    last_mtr_values.current_errors.dc_bus_over_current      = (input >> 9)  & 0x1;
    last_mtr_values.current_errors.dc_bus_over_regen_current= (input >> 10) & 0x1;
    last_mtr_values.current_errors.current_limit_violation  = (input >> 11) & 0x1;
    last_mtr_values.current_errors.motor_over_temp          = (input >> 12) & 0x1;
    last_mtr_values.current_errors.inverter_over_temp       = (input >> 13) & 0x1;
    last_mtr_values.current_errors.velocity_limit_violation = (input >> 14) & 0x1;
    last_mtr_values.current_errors.position_limit_violation = (input >> 15) & 0x1;
    last_mtr_values.current_errors.watchdog_timer_expired   = (input >> 16) & 0x1;
    last_mtr_values.current_errors.estop_requested          = (input >> 17) & 0x1;
    last_mtr_values.current_errors.spinout_detected         = (input >> 18) & 0x1;
    last_mtr_values.current_errors.brake_resistor_disarmed  = (input >> 19) & 0x1;
    last_mtr_values.current_errors.thermistor_disconnected  = (input >> 20) & 0x1;
    last_mtr_values.current_errors.calibration_error        = (input >> 21) & 0x1;
}

SRT_OdriveMtr::~SRT_OdriveMtr() {}

