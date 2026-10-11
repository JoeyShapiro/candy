/*
 * twai_sniffer.c
 *
 * Passive CAN sniffer for the XIAO ESP32-C3 + SN65HVD230 transceiver:
 *
 *   SN65HVD230 CTX -> GPIO4 (D2)
 *   SN65HVD230 CRX -> GPIO5 (D3)
 *   SN65HVD230 CANH/CANL -> OBD-II pins 6/14
 *   SN65HVD230 GND -> XIAO GND and OBD-II pin 5 (signal ground)
 *
 * Runs the controller in listen-only mode: it never transmits, not even
 * ACK bits or error frames, so it can't disturb the car's bus. Every
 * frame on the bus is printed as raw hex:
 *
 *     12.345678 RX 17C  8  00 00 02 F1 00 00 00 11
 *
 * Decode the log afterwards with little-win.py and honda.dbc.
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/twai.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_err.h"

#define TWAI_TX_GPIO GPIO_NUM_4
#define TWAI_RX_GPIO GPIO_NUM_5

#define STATUS_PERIOD_MS 5000

#define ARRAY_LEN(a) (sizeof(a) / sizeof((a)[0]))

static double now_s(void)
{
    return esp_timer_get_time() / 1e6;
}

static void print_raw(const twai_message_t *msg)
{
    printf("%12.6f RX %03lX  %d ", now_s(), (unsigned long)msg->identifier, msg->data_length_code);
    for (int i = 0; i < msg->data_length_code && i < 8; i++) {
        printf(" %02X", msg->data[i]);
    }
    printf("\n");
}

static void print_status(void)
{
    static const char *state_names[] = { "STOPPED", "RUNNING", "BUS_OFF", "RECOVERING" };
    twai_status_info_t status;
    esp_err_t err = twai_get_status_info(&status);
    if (err != ESP_OK) {
        printf("# status: twai_get_status_info failed: %s\n", esp_err_to_name(err));
        return;
    }
    printf("# status: state=%s rx_err=%lu rx_queued=%lu bus_err=%lu rx_missed=%lu rx_overrun=%lu\n",
           status.state < ARRAY_LEN(state_names) ? state_names[status.state] : "?",
           (unsigned long)status.rx_error_counter, (unsigned long)status.msgs_to_rx,
           (unsigned long)status.bus_error_count, (unsigned long)status.rx_missed_count,
           (unsigned long)status.rx_overrun_count);
}

void app_main(void)
{
    /* Give the USB serial console time to reconnect after reset so the
     * first lines aren't lost. */
    vTaskDelay(pdMS_TO_TICKS(2000));

    twai_general_config_t g_config =
        TWAI_GENERAL_CONFIG_DEFAULT(TWAI_TX_GPIO, TWAI_RX_GPIO, TWAI_MODE_LISTEN_ONLY);
    /* The default 5-frame queue overflows within milliseconds on a busy bus. */
    g_config.rx_queue_len = 128;
    /* No alerts: the driver would otherwise log some of them from its ISR. */
    g_config.alerts_enabled = TWAI_ALERT_NONE;
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
    if (err != ESP_OK) {
        printf("# twai_driver_install failed: %s\n", esp_err_to_name(err));
        return;
    }
    err = twai_start();
    if (err != ESP_OK) {
        printf("# twai_start failed: %s\n", esp_err_to_name(err));
        return;
    }
    printf("# listen-only sniffer at 500kbps, RX=GPIO%d\n", TWAI_RX_GPIO);

    int64_t last_status_us = esp_timer_get_time();
    while (1) {
        twai_message_t msg;
        if (twai_receive(&msg, pdMS_TO_TICKS(STATUS_PERIOD_MS)) == ESP_OK) {
            print_raw(&msg);
        }
        if (esp_timer_get_time() - last_status_us >= STATUS_PERIOD_MS * 1000LL) {
            last_status_us = esp_timer_get_time();
            print_status();
        }
    }
}
