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
#include <string.h>
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

/* The C3 has no PSRAM and ~400 KB of SRAM total, so this lives in .bss
 * where an oversized buffer fails at link time rather than at runtime. */
#define CAPTURE_BUF_BYTES (128 * 1024)

/* 16 bytes per frame. t_us wraps every ~71 minutes; id only holds
 * standard 11-bit IDs, so extended frames are rejected in capture(). */
typedef struct {
    uint32_t t_us;
    uint16_t id;
    uint8_t dlc;
    uint8_t data[8];
} capture_t;

_Static_assert(sizeof(capture_t) == 16, "capture_t should pack to 16 bytes");

static capture_t s_capture[CAPTURE_BUF_BYTES / sizeof(capture_t)];
static size_t s_capture_len;
static uint32_t s_extd_dropped;

static void capture(const twai_message_t *msg)
{
    if (msg->extd || msg->identifier > 0x7FF) {
        /* Only report the first one; the count goes in the status line so
         * a bus full of extended frames can't flood the console. */
        if (s_extd_dropped++ == 0) {
            printf("# error: 29-bit ID %08lX doesn't fit in capture_t, dropping extended frames\n",
                   (unsigned long)msg->identifier);
        }
        return;
    }
    if (s_capture_len >= ARRAY_LEN(s_capture)) {
        return;
    }
    capture_t *c = &s_capture[s_capture_len++];
    c->t_us = (uint32_t)esp_timer_get_time();
    c->id = (uint16_t)msg->identifier;
    c->dlc = msg->data_length_code;
    memcpy(c->data, msg->data, sizeof(c->data));
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
    printf("# status: state=%s rx_err=%lu rx_queued=%lu bus_err=%lu rx_missed=%lu rx_overrun=%lu captured=%u/%u extd_dropped=%lu\n",
           status.state < ARRAY_LEN(state_names) ? state_names[status.state] : "?",
           (unsigned long)status.rx_error_counter, (unsigned long)status.msgs_to_rx,
           (unsigned long)status.bus_error_count, (unsigned long)status.rx_missed_count,
           (unsigned long)status.rx_overrun_count,
           (unsigned)s_capture_len, (unsigned)ARRAY_LEN(s_capture),
           (unsigned long)s_extd_dropped);
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
            capture(&msg);
        }
        if (esp_timer_get_time() - last_status_us >= STATUS_PERIOD_MS * 1000LL) {
            last_status_us = esp_timer_get_time();
            print_status();
        }
    }
}
