#include "driver/twai.h"
#include <ODriveCan.h>

// ---- fwd decls ----
int  send_can_msg(uint16_t can_id, uint8_t len, uint8_t* data, bool rtr);
bool init_twai(gpio_num_t tx, gpio_num_t rx);
void can_check_recv();
bool wait_axis_idle_no_error(uint32_t ms_timeout);
bool wait_closed_loop(uint32_t ms_timeout);
bool calibrate_and_save();
void init_motor();

// Node ID 0 (avoid 0x3F broadcast)
ODriveCanMtr mtr1(&send_can_msg, 0);

void setup() {
  Serial.begin(115200);

  // GIM default bitrate: 500 kbit/s
  if (!init_twai((gpio_num_t)5, (gpio_num_t)4)) { // TX, RX pins – adjust for your board
    Serial.println("TWAI init failed");
    while (1) delay(1000);
  }

  mtr1.begin();
  Serial.println("CAN up, ODrive wrapper begun");

  init_motor();

  // Conservative limits first (turns/s, A)
  mtr1.set_lim(5.0f, 2.0f);

  // Example target velocity with small torque FF
  mtr1.set_ip_vel(3.0f, 0.5f);
  Serial.println("Commanded velocity");
}

void loop() {
  can_check_recv();

  if (!mtr1.mtr_connected()) {
    Serial.println("Heartbeat timeout: re-initializing axis");
    init_motor();
    mtr1.set_lim(5.0f, 2.0f);
    mtr1.set_ip_vel(3.0f, 0.5f);
  }

  // (Optional) poll a bit
  delay(5);
}

// --------- CAN TX ----------
int send_can_msg(uint16_t can_id, uint8_t len, uint8_t* data, bool rtr)
{
  twai_message_t msg = {};
  msg.identifier = can_id;
  msg.extd       = 0;             // standard 11-bit
  msg.rtr        = rtr ? 1 : 0;   // set for info (RTR) frames
  msg.self       = 0;
  msg.ss         = 0;

  if (len > 8) len = 8;           // classic CAN
  msg.data_length_code = len;

  if (!rtr && len > 0 && data != nullptr) {
    memcpy(msg.data, data, len);
  }

  if (twai_transmit(&msg, pdMS_TO_TICKS(500)) == ESP_OK) {
    return success;               // from your proccess_return enum
  } else {
    return can_transmission_fail;
  }
}

// --------- CAN init ----------
bool init_twai(gpio_num_t tx, gpio_num_t rx)
{
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(tx, rx, TWAI_MODE_NORMAL);
  g_config.rx_queue_len = 20;

  // GIM default: 500 kbit/s
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
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

// --------- RX pump ----------
void can_check_recv()
{
  twai_message_t msg;
  while (twai_receive(&msg, 0) == ESP_OK) {
    // Only parse data frames (RTR have no payload to parse)
    if (!msg.rtr) {
      mtr1.process_msg(msg.identifier, msg.data_length_code, msg.data);
    }
  }
}

// Wait until axis reports IDLE and no axis_error for up to ms_timeout
bool wait_axis_idle_no_error(uint32_t ms_timeout)
{
  uint32_t t0 = millis();
  while (millis() - t0 < ms_timeout) {
    can_check_recv();
    if (mtr1.last_mtr_values.axis_state == idle &&
        mtr1.last_mtr_values.axis_error == 0) {
      return true;
    }
    delay(20);
  }
  return false;
}

// Wait until axis reaches CLOSED_LOOP_CONTROL (and no axis_error)
bool wait_closed_loop(uint32_t ms_timeout)
{
  uint32_t t0 = millis();
  while (millis() - t0 < ms_timeout) {
    can_check_recv();
    if (mtr1.last_mtr_values.axis_state == closed_loop_control &&
        mtr1.last_mtr_values.axis_error == 0) {
      return true;
    }
    if (mtr1.last_mtr_values.axis_error != 0) return false;
    delay(20);
  }
  return false;
}

// Run calibration once, save to flash, reboot, and wait for heartbeats
bool calibrate_and_save()
{
  Serial.println("Calibration: starting full_calibration_sequence");
  if (mtr1.set_axis_state(full_calibration_sequence) != success) {
    Serial.println("Calibration: cmd tx failed");
    return false;
  }

  // Give it time to run the sequence (spins motor)
  if (!wait_axis_idle_no_error(15000)) { // 15 s window is usually ample
    Serial.print("Calibration: failed. axis_error=0x");
    Serial.println(mtr1.last_mtr_values.axis_error, HEX);
    return false;
  }

  Serial.println("Calibration OK. Saving config & rebooting...");
  if (mtr1.reboot_mtr(a_save_configuration) != success) {
    Serial.println("Save+reboot command failed");
    return false;
  }

  // Wait for device to reboot and resume heartbeat
  delay(1500);
  uint32_t t0 = millis();
  while (millis() - t0 < 3000) { // 3 s to see heartbeats again
    can_check_recv();
    if (mtr1.mtr_connected()) break;
    delay(20);
  }
  return mtr1.mtr_connected();
}

// Try closed loop; if it fails, calibrate, save, reboot, and try again
void init_motor()
{
  // Controller: velocity + vel ramp is a sane default
  mtr1.set_cont_mode(velocity_control, vel_ramp);
  delay(20);

  // First attempt: go straight to closed loop (works if already calibrated/saved)
  mtr1.set_axis_state(closed_loop_control);
  if (wait_closed_loop(1000)) {
    Serial.println("Entered closed loop (pre-calibrated).");
    return;
  }

  Serial.println("Closed loop failed; running calibration...");
  if (!calibrate_and_save()) {
    Serial.println("Calibration sequence failed.");
    return;
  }

  // After reboot, set mode and enter closed loop again
  mtr1.set_cont_mode(velocity_control, vel_ramp);
  delay(20);
  mtr1.set_axis_state(closed_loop_control);
  if (wait_closed_loop(2000)) {
    Serial.println("Closed loop after calibration.");
  } else {
    Serial.print("Closed loop still failing. axis_error=0x");
    Serial.println(mtr1.last_mtr_values.axis_error, HEX);
  }
}
