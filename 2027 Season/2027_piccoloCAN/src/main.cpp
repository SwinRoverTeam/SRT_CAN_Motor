#include <Arduino.h>
#include "driver/twai.h"
#include "SRT_PiccoloCAN.h"

// ---- Config ----
#define CAN_TX_PIN   GPIO_NUM_18     // to transceiver TXD (e.g. SN65HVD230 / TJA1051)
#define CAN_RX_PIN   GPIO_NUM_16     // to transceiver RXD
#define SERVO_ID     14              // servo CAN node ID (1..254; 0 = disabled)

// ---- TWAI send wrapper (extended frames) ----
int can_send_ext(uint32_t id, uint8_t len, uint8_t *data) {
    twai_message_t msg = {};
    msg.extd = 1;                   // 29-bit ID (CAN 2.0B)
    msg.rtr = 0;
    msg.identifier = id;
    msg.data_length_code = len;
    memcpy(msg.data, data, len);
    return (twai_transmit(&msg, pdMS_TO_TICKS(5)) == ESP_OK) ? 0 : -1;
}

SRT_PiccoloServo servo(can_send_ext, SERVO_ID);

bool can_begin() {
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    g.tx_queue_len = 10;
    g.rx_queue_len = 20;
    twai_timing_config_t t = TWAI_TIMING_CONFIG_1MBITS();     // servo runs 1 Mbit/s
    twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL(); // filter in software
    if (twai_driver_install(&g, &t, &f) != ESP_OK) return false;
    return twai_start() == ESP_OK;
}

void can_poll_rx() {
    twai_message_t rx;
    while (twai_receive(&rx, 0) == ESP_OK) {
        if (!rx.extd || rx.rtr) continue;
        servo.process_msg(rx.identifier, rx.data_length_code, rx.data);
    }
}

// Recover from bus-off
void can_check_bus() {
    twai_status_info_t s;
    twai_get_status_info(&s);
    if (s.state == TWAI_STATE_BUS_OFF) twai_initiate_recovery();
    else if (s.state == TWAI_STATE_STOPPED) twai_start();
}

void setup() {
    Serial.begin(115200);
    delay(500);
    if (!can_begin()) {
        Serial.println("TWAI init failed");
        while (true) delay(1000);
    }
    Serial.println("TWAI started @ 1 Mbit/s");

    servo.enable();
    delay(20);
}

void loop() {
    static uint32_t t_cmd = 0, t_poll = 0, t_print = 0;
    static int16_t pos = 0;
    static int16_t step = 200;
    uint32_t now = millis();

    can_poll_rx();
    can_check_bus();

    // Position command at 50 Hz (servo may time out and hold/disable if commands stop)
    if (now - t_cmd >= 20) {
        t_cmd = now;
        pos += step;
        if (pos >= 10000 || pos <= -10000) step = -step;
        servo.set_position(pos);
    }

    // Poll status A + B at 10 Hz
    if (now - t_poll >= 100) {
        t_poll = now;
        servo.request_status_a();
        servo.request_status_b();
    }

    // Print at 2 Hz
    if (now - t_print >= 500) {
        t_print = now;
        const PiccoloStatusA &a = servo.status_a();
        const PiccoloStatusB &b = servo.status_b();
        if (a.valid) {
            Serial.printf("en=%d cmdRx=%d pos=%d cmd=%d warn=0x%02X err=0x%02X | ",
                          a.enabled, a.cmd_received, a.position, a.command, a.warnings, a.errors);
        } else {
            Serial.print("no StatusA | ");
        }
        if (b.valid) {
            Serial.printf("%umA %umV %dC duty=%d%% %ddps\n",
                          b.current_mA, b.voltage_mV, b.temperature_C, b.duty_pct, b.speed_dps);
        } else {
            Serial.println("no StatusB");
        }
    }
}