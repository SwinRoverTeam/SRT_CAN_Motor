#include "driver/twai.h"
#include <ODriveCan.h>

int send_can_msg(uint16_t can_id, uint8_t len, uint8_t* data, bool rtr);

ODriveCanMtr mtr1(&send_can_msg, 0);


void setup() {
  Serial.begin(115200);
  // put your setup code here, to run once:
  init_twai((gpio_num_t) 4, (gpio_num_t) 5);
  mtr1.begin();
  Serial.println("Began motor");
  init_motor();
  Serial.println("Initted motor");
  mtr1.set_lim(10, 3);
  mtr1.set_ip_vel(5, 1);
  Serial.println("Set motor velocity");
}

void loop() {
  can_check_recv();
  
  if (!mtr1.mtr_connected()) {
    Serial.println("Motor disconnected");
    init_motor();
    mtr1.set_lim(10, 3);
    mtr1.set_ip_vel(5, 1);
  }
}

int send_can_msg(uint16_t can_id, uint8_t len, uint8_t* data, bool rtr)
{
  twai_message_t msg = {
    msg.rtr = (int) rtr,
    msg.extd = 0,
    msg.identifier = can_id,
    msg.self = 0,
    msg.ss = 0
  };

  if (len <= 8) {
    msg.dlc_non_comp = 0;
  } else {
    msg.dlc_non_comp = 1;
  }
  msg.data_length_code = len;
  memcpy(msg.data, data, len);

  if (twai_transmit(&msg, pdMS_TO_TICKS(500)) == ESP_OK) {
    return success;
  } else {
    return can_transmission_fail;
  }
}

bool init_twai(gpio_num_t rx, gpio_num_t tx)
{
  // Initialize configuration structures using macro initializers
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(tx, rx, TWAI_MODE_NORMAL);
  g_config.rx_queue_len = 10;

  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    printf("Driver installed\n");
  } else {
    printf("Failed to install driver\n");
    return false;
  }

  // Start TWAI driver
  if (twai_start() == ESP_OK) {
    printf("Driver started\n");
  } else {
    printf("Failed to start driver\n");
    return false;
  }
  
  return true;
}

void can_check_recv()
{
  twai_message_t msg;
  while (twai_receive(&msg, 500) == ESP_OK) {
    mtr1.process_msg(msg.identifier, msg.data_length_code, msg.data);
  }
}

void init_motor()
{
  mtr1.set_cont_mode(velocity_control, vel_ramp);
  delay(600);
  mtr1.set_axis_state(closed_loop_control);
}