/*
 * twai_sniffer.c
 *
 * Minimal TWAI (CAN) bus reader for the XIAO ESP32-C3, wired to an
 * SN65HVD230 transceiver:
 *
 *   SN65HVD230 TXD -> GPIO4 (D2)
 *   SN65HVD230 RXD -> GPIO5 (D3)
 *   SN65HVD230 CANH/CANL -> OBD-II pins 6/14
 *
 * Listens at 500kbps (the standard high-speed CAN rate most gasoline
 * cars use for the powertrain/diagnostic bus) and prints every frame
 * it sees over the USB-serial console: ID, data length, and raw
 * bytes in hex. No filtering yet -- this is step one, just proving
 * the wiring and driver init are correct.
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/twai.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define TWAI_TX_GPIO GPIO_NUM_4
#define TWAI_RX_GPIO GPIO_NUM_5

static const char *TAG = "twai_sniffer";

#define HEARTBEAT_MS 5000

void app_main(void)
{
    /* The C3's USB-serial console only exists once the host re-enumerates
     * it after reset, so anything logged in the first moment of boot is
     * usually gone before `pio device monitor` attaches. Wait a bit so
     * the startup messages actually show up. */
    vTaskDelay(pdMS_TO_TICKS(2000));
    ESP_LOGI(TAG, "twai_sniffer booting");

    twai_general_config_t g_config =
        TWAI_GENERAL_CONFIG_DEFAULT(TWAI_TX_GPIO, TWAI_RX_GPIO, TWAI_MODE_NORMAL);

    /* 500kbps is the standard high-speed CAN bitrate on the OBD-II
     * diagnostic bus for CAN-based vehicles (mandatory in the US from
     * MY2008 onward). If you get silence, this is the first thing to
     * suspect -- some manufacturers put a slower ~125kbps bus on
     * body/comfort systems, separate from the high-speed powertrain
     * bus that's usually what's wired to pins 6/14. */
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();

    /* Accept every frame for now -- no filtering. Once you know which
     * IDs you care about, replace this with a real acceptance filter
     * so you're not parsing/printing bus-load traffic you don't need. */
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "twai_driver_install failed: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "TWAI driver installed (TX=GPIO%d, RX=GPIO%d)", TWAI_TX_GPIO, TWAI_RX_GPIO);

    err = twai_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "twai_start failed: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "TWAI started at 500kbps, listening...");

    uint32_t frames_total = 0;
    uint32_t frames_since_heartbeat = 0;
    TickType_t last_heartbeat = xTaskGetTickCount();

    while (1) {
        twai_message_t message;

        /* Block for up to 1s waiting for a frame. A 1s timeout with no
         * message is normal if the car is off/asleep -- see the notes
         * about waking the bus (key in ACC/ON) from earlier. */
        esp_err_t res = twai_receive(&message, pdMS_TO_TICKS(1000));

        if ((xTaskGetTickCount() - last_heartbeat) >= pdMS_TO_TICKS(HEARTBEAT_MS)) {
            twai_status_info_t status;
            twai_get_status_info(&status);
            ESP_LOGI(TAG, "alive: %lu frames in last %ds (%lu total), state=%d rx_err=%lu bus_err=%lu rx_missed=%lu",
                     (unsigned long)frames_since_heartbeat, HEARTBEAT_MS / 1000,
                     (unsigned long)frames_total, status.state,
                     (unsigned long)status.rx_error_counter,
                     (unsigned long)status.bus_error_count,
                     (unsigned long)status.rx_missed_count);
            frames_since_heartbeat = 0;
            last_heartbeat = xTaskGetTickCount();
        }

        if (res == ESP_OK) {
            frames_total++;
            frames_since_heartbeat++;
            printf("ID: 0x%03lX  DLC: %d  Data:",
                   (unsigned long)message.identifier,
                   message.data_length_code);

            for (int i = 0; i < message.data_length_code; i++) {
                printf(" %02X", message.data[i]);
            }
            printf("\n");

        } else if (res == ESP_ERR_TIMEOUT) {
            /* Nothing received in the last second -- keep listening. */
            continue;

        } else {
            ESP_LOGW(TAG, "twai_receive error: %s", esp_err_to_name(res));

            /* Bus errors (e.g. from a wiring problem, or the receiver
             * going into a bus-off state after too many errors) show up
             * here. Check twai_get_status_info() if this fires
             * repeatedly -- it'll tell you if the controller has
             * dropped into an error-passive or bus-off state. */
        }
    }
}
