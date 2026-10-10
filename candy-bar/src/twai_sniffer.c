/*
 * twai_sniffer.c
 *
 * CAN bus logger for the XIAO ESP32-C3 + SN65HVD230 transceiver:
 *
 *   SN65HVD230 CTX -> GPIO4 (D2)
 *   SN65HVD230 CRX -> GPIO5 (D3)
 *   SN65HVD230 CANH/CANL -> OBD-II pins 6/14
 *   SN65HVD230 GND -> XIAO GND and OBD-II pin 5 (signal ground)
 *
 * Logs every frame on the bus as raw hex with a timestamp (seconds
 * since boot):
 *
 *       12.345678 RX 7E8  8  06 41 00 BE 3F A8 13 00
 *
 * Once a second it sends the OBD-II request "mode 01 PID 00" (which
 * PIDs do you support?) -- every OBD-II car must answer it. Replies
 * from the engine/transmission ECUs (IDs 7E8-7EF) get an extra decoded
 * line starting with "#". Nothing else is decoded: the rest of the
 * traffic is manufacturer-specific.
 *
 * If a request ever fails to send, it stops sending until reboot and
 * just keeps listening, so a wiring problem can't keep disturbing the
 * car's bus.
 */

#include <stdio.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/twai.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#define TWAI_TX_GPIO GPIO_NUM_4
#define TWAI_RX_GPIO GPIO_NUM_5

/* Set to 0 to only listen. */
#define SEND_OBD_REQUEST 1
#define REQUEST_PERIOD_MS 1000

#define STATUS_PERIOD_MS 5000

static void print_frame(const char *dir, const twai_message_t *msg, int64_t t_us)
{
    printf("%6lld.%06lld %s ", t_us / 1000000, t_us % 1000000, dir);
    if (msg->extd) {
        printf("%08lX", (unsigned long)msg->identifier);
    } else {
        printf("%03lX", (unsigned long)msg->identifier);
    }
    printf("  %d ", msg->data_length_code);
    for (int i = 0; i < msg->data_length_code && i < 8; i++) {
        printf(" %02X", msg->data[i]);
    }
    printf("\n");
}

/* Decode standard OBD-II replies. Byte 0 is the length of what follows,
 * byte 1 the response mode (request mode + 0x40), byte 2 the PID. */
static void decode_obd_reply(const twai_message_t *msg)
{
    if (msg->extd || msg->identifier < 0x7E8 || msg->identifier > 0x7EF || msg->data_length_code < 3) {
        return;
    }
    const uint8_t *d = msg->data;
    int ecu = msg->identifier - 0x7E8;

    if (d[1] == 0x7F) {
        printf("#   ECU %d: request for mode %02X refused (code %02X)\n", ecu, d[2], d[3]);
        return;
    }
    if (d[1] != 0x41 || d[0] < 6) {
        return;
    }

    /* PIDs 00, 20, 40, ... are 32-bit bitmaps: bit 31 = PID+1, bit 0 = PID+32. */
    uint8_t pid = d[2];
    if (pid % 0x20 == 0) {
        uint32_t bits = ((uint32_t)d[3] << 24) | ((uint32_t)d[4] << 16) | ((uint32_t)d[5] << 8) | d[6];
        printf("#   ECU %d supports mode 01 PIDs:", ecu);
        for (int i = 0; i < 32; i++) {
            if (bits & (1UL << (31 - i))) {
                printf(" %02X", pid + 1 + i);
            }
        }
        printf("\n");
    }
}

void app_main(void)
{
    /* Give the USB serial console time to reconnect after reset so the
     * first lines aren't lost. */
    vTaskDelay(pdMS_TO_TICKS(2000));

    twai_general_config_t g_config =
        TWAI_GENERAL_CONFIG_DEFAULT(TWAI_TX_GPIO, TWAI_RX_GPIO, TWAI_MODE_NORMAL);
    /* The default 5-frame queue overflows within milliseconds on a busy bus. */
    g_config.rx_queue_len = 128;
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK || twai_start() != ESP_OK) {
        printf("# failed to start TWAI driver\n");
        return;
    }
    printf("# listening at 500kbps, TX=GPIO%d RX=GPIO%d\n", TWAI_TX_GPIO, TWAI_RX_GPIO);
    printf("# format: <seconds since boot> <RX|TX> <id> <dlc> <data bytes>\n");

    bool sending = SEND_OBD_REQUEST;
    uint32_t rx_count = 0;
    int64_t last_request_us = 0;
    int64_t last_status_us = esp_timer_get_time();

    while (1) {
        twai_message_t msg;
        if (twai_receive(&msg, pdMS_TO_TICKS(100)) == ESP_OK) {
            print_frame("RX", &msg, esp_timer_get_time());
            decode_obd_reply(&msg);
            rx_count++;
        }

        int64_t now = esp_timer_get_time();
        twai_status_info_t status;
        twai_get_status_info(&status);

        if (sending && status.tx_error_counter > 0) {
            /* A healthy bus never produces transmit errors. */
            sending = false;
            printf("# a request failed to send (tx_err=%lu) -- no more requests until reboot, still listening\n",
                   (unsigned long)status.tx_error_counter);
        }

        if (sending && now - last_request_us >= REQUEST_PERIOD_MS * 1000LL) {
            last_request_us = now;
            /* 7DF = "all OBD ECUs". Data: 2 bytes follow, mode 01, PID 00. */
            twai_message_t req = {
                .identifier = 0x7DF,
                .data_length_code = 8,
                .data = { 0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
            };
            /* Single-shot: on failure, don't let the controller retry it
             * over and over onto the car's bus. */
            req.ss = 1;
            if (twai_transmit(&req, 0) == ESP_OK) {
                print_frame("TX", &req, now);
            }
        }

        if (now - last_status_us >= STATUS_PERIOD_MS * 1000LL) {
            last_status_us = now;
            printf("# status: %lu frames received, state=%d tx_err=%lu rx_err=%lu bus_err=%lu rx_missed=%lu\n",
                   (unsigned long)rx_count, status.state,
                   (unsigned long)status.tx_error_counter, (unsigned long)status.rx_error_counter,
                   (unsigned long)status.bus_error_count, (unsigned long)status.rx_missed_count);

            /* Bus-off (too many errors) silences the controller for good
             * unless recovered; recovery ends in STOPPED, then restart. */
            if (status.state == TWAI_STATE_BUS_OFF) {
                sending = false;
                twai_initiate_recovery();
            } else if (status.state == TWAI_STATE_STOPPED) {
                twai_clear_transmit_queue();
                twai_start();
            }
        }
    }
}
