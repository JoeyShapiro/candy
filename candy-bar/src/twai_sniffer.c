/*
 * twai_sniffer.c
 *
 * Basic OBD-II reader for the XIAO ESP32-C3 + SN65HVD230 transceiver:
 *
 *   SN65HVD230 CTX -> GPIO4 (D2)
 *   SN65HVD230 CRX -> GPIO5 (D3)
 *   SN65HVD230 CANH/CANL -> OBD-II pins 6/14
 *   SN65HVD230 GND -> XIAO GND and OBD-II pin 5 (signal ground)
 *
 * Asks the car for one value at a time (RPM, speed, temps, ...) with a
 * standard mode 01 request, waits for the reply, and prints it decoded:
 *
 *       12.345678 RPM                  812.50 rpm      [7E8: 04 41 0C 0C B2]
 *
 * Decoding formulas follow python-OBD (obd/commands.py, decoders.py).
 * All the car's own manufacturer-specific traffic is ignored unless
 * LOG_ALL_FRAMES is set.
 *
 * Requests only start once the bus is active (a sleeping bus can't ACK
 * them). After a failed send it waits for activity again; after
 * MAX_CONSECUTIVE_FAILS failures in a row (or bus-off) it stops sending
 * until reboot, so a wiring problem can't keep disturbing the car's bus.
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

/* Set to 1 to also print every other frame on the bus as raw hex. */
#define LOG_ALL_FRAMES 0

/* ECUs must answer within 50ms; give them double. */
#define REPLY_TIMEOUT_MS 100
/* Pause after each full pass through the PID list. */
#define CYCLE_PAUSE_MS 500

#define STATUS_PERIOD_MS 5000

/* Failed sends in a row before giving up until reboot. */
#define MAX_CONSECUTIVE_FAILS 5

/*
 * A mode 01 PID and how to turn its reply bytes into a value:
 *   value = (first `bytes` data bytes, big-endian) * scale + offset
 * which covers python-OBD's percent, temp and simple UAS decoders.
 */
typedef struct {
    uint8_t pid;
    const char *name;
    uint8_t bytes;
    float scale;
    float offset;
    const char *unit;
} obd_pid_t;

static const obd_pid_t pids[] = {
    { 0x0C, "RPM",           2, 0.25f,          0.0f,   "rpm"  },
    { 0x0D, "SPEED",         1, 1.0f,           0.0f,   "km/h" },
    { 0x05, "COOLANT_TEMP",  1, 1.0f,           -40.0f, "C"    },
    { 0x04, "ENGINE_LOAD",   1, 100.0f / 255.0f, 0.0f,  "%"    },
    { 0x11, "THROTTLE_POS",  1, 100.0f / 255.0f, 0.0f,  "%"    },
    { 0x0F, "INTAKE_TEMP",   1, 1.0f,           -40.0f, "C"    },
    { 0x10, "MAF",           2, 0.01f,          0.0f,   "g/s"  },
    { 0x2F, "FUEL_LEVEL",    1, 100.0f / 255.0f, 0.0f,  "%"    },
    { 0x42, "MODULE_VOLTAGE", 2, 0.001f,        0.0f,   "V"    },
};

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

/* Is this a reply to our request for `pid`? Engine/transmission ECUs
 * answer on 7E8-7EF; the payload is <length> 41 <pid> <data...>. */
static bool is_reply(const twai_message_t *msg, uint8_t pid)
{
    return !msg->extd && msg->identifier >= 0x7E8 && msg->identifier <= 0x7EF &&
           msg->data_length_code >= 3 && msg->data[1] == 0x41 && msg->data[2] == pid;
}

static void print_value(const obd_pid_t *p, const twai_message_t *msg)
{
    const uint8_t *d = msg->data;
    int len = d[0];   /* bytes after the length byte: 41, pid, data... */
    printf("%12.6f %-15s ", now_s(), p->name);
    if (len - 2 < p->bytes || 3 + p->bytes > msg->data_length_code) {
        printf("short reply");
    } else {
        uint32_t raw = 0;
        for (int i = 0; i < p->bytes; i++) {
            raw = (raw << 8) | d[3 + i];
        }
        printf("%10.2f %-5s", raw * p->scale + p->offset, p->unit);
    }
    printf("  [%03lX:", (unsigned long)msg->identifier);
    for (int i = 0; i <= len && i < 8; i++) {
        printf(" %02X", d[i]);
    }
    printf("]\n");
}

/* Send a mode 01 request for `pid` to all OBD ECUs (7DF).
 * Single-shot: if it fails, the controller doesn't retry it. */
static bool send_request(uint8_t pid)
{
    twai_message_t req = {
        .identifier = 0x7DF,
        .data_length_code = 8,
        .data = { 0x02, 0x01, pid, 0x00, 0x00, 0x00, 0x00, 0x00 },
    };
    req.ss = 1;
    return twai_transmit(&req, pdMS_TO_TICKS(REPLY_TIMEOUT_MS)) == ESP_OK;
}

/* Receive until the reply to `pid` arrives or the timeout passes. Other
 * frames are dropped (or printed raw with LOG_ALL_FRAMES). */
static bool wait_for_reply(const obd_pid_t *p)
{
    int64_t deadline = esp_timer_get_time() + REPLY_TIMEOUT_MS * 1000LL;
    while (esp_timer_get_time() < deadline) {
        twai_message_t msg;
        if (twai_receive(&msg, pdMS_TO_TICKS(10)) != ESP_OK) {
            continue;
        }
        if (is_reply(&msg, p->pid)) {
            print_value(p, &msg);
            return true;
        }
        if (LOG_ALL_FRAMES) {
            print_raw(&msg);
        }
    }
    return false;
}

static void print_status(void)
{
    twai_status_info_t status;
    twai_get_status_info(&status);
    printf("# status: state=%d tx_err=%lu rx_err=%lu bus_err=%lu rx_missed=%lu\n",
           status.state, (unsigned long)status.tx_error_counter,
           (unsigned long)status.rx_error_counter, (unsigned long)status.bus_error_count,
           (unsigned long)status.rx_missed_count);
}

/* Wait until the car is talking -- a request on a sleeping bus can't be ACKed. */
static void wait_for_bus_activity(void)
{
    printf("# waiting for bus activity (turn ignition ON)...\n");
    twai_message_t msg;
    while (twai_receive(&msg, pdMS_TO_TICKS(1000)) != ESP_OK) { }
    /* Drain whatever queued up so stale frames don't confuse wait_for_reply. */
    while (twai_receive(&msg, 0) == ESP_OK) { }
    printf("# bus active, starting requests\n");
}

/* Stop sending for good, but keep draining RX so rx_missed stays meaningful. */
static void halt_sending(void)
{
    printf("# repeated TX failures -- stopping (reboot to retry), still draining RX\n");
    print_status();
    int64_t last_status_us = esp_timer_get_time();
    while (1) {
        twai_message_t msg;
        if (twai_receive(&msg, pdMS_TO_TICKS(STATUS_PERIOD_MS)) == ESP_OK && LOG_ALL_FRAMES) {
            print_raw(&msg);
        }
        if (esp_timer_get_time() - last_status_us >= STATUS_PERIOD_MS * 1000LL) {
            last_status_us = esp_timer_get_time();
            print_status();
        }
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
    printf("# OBD-II reader at 500kbps, TX=GPIO%d RX=GPIO%d\n", TWAI_TX_GPIO, TWAI_RX_GPIO);

    twai_reconfigure_alerts(TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_FAILED |
                        TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_OFF, NULL);

    wait_for_bus_activity();

    int64_t last_status_us = esp_timer_get_time();
    int consecutive_fails = 0;

    while (1) {
        for (size_t i = 0; i < ARRAY_LEN(pids); i++) {
            const obd_pid_t *p = &pids[i];

            if (!send_request(p->pid)) {
                printf("# could not queue request for %s\n", p->name);
            } else if (!wait_for_reply(p)) {
                printf("%12.6f %-15s no reply\n", now_s(), p->name);
            }

            /* The TX error counter only drops by one per successful send, so
             * judge each attempt by its alert rather than by the counter. */
            uint32_t alerts = 0;
            twai_read_alerts(&alerts, pdMS_TO_TICKS(50));
            if (alerts & TWAI_ALERT_TX_SUCCESS) printf("TX acked by a node\n");
            if (alerts & TWAI_ALERT_TX_FAILED)  printf("TX failed (no ACK or bit error)\n");

            twai_status_info_t status;
            twai_get_status_info(&status);
            if (status.state == TWAI_STATE_BUS_OFF) {
                halt_sending();
            }
            if ((alerts & TWAI_ALERT_TX_FAILED) || status.state != TWAI_STATE_RUNNING) {
                print_status();
                if (++consecutive_fails >= MAX_CONSECUTIVE_FAILS) {
                    halt_sending();
                }
                /* Probably asleep -- wait for traffic, then retry. */
                wait_for_bus_activity();
            } else if (alerts & TWAI_ALERT_TX_SUCCESS) {
                consecutive_fails = 0;
            }
        }

        if (esp_timer_get_time() - last_status_us >= STATUS_PERIOD_MS * 1000LL) {
            last_status_us = esp_timer_get_time();
            print_status();
        }
        vTaskDelay(pdMS_TO_TICKS(CYCLE_PAUSE_MS));
    }
}
