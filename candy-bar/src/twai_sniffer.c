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
#include "esp_err.h"

#define TWAI_TX_GPIO GPIO_NUM_4
#define TWAI_RX_GPIO GPIO_NUM_5

/* ESP32-C3 TWAI controller: base 0x6002B000, error code capture
 * register at offset 0x30 (SJA1000 register 12). Check the base against
 * the "TWAI" entry in the ESP32-C3 Technical Reference Manual's
 * peripheral address map if the values look like nonsense. */
#define TWAI_ECC_REG (*(volatile uint32_t *)(0x6002B000 + 0x30))
static void print_last_error(void)
{
    uint32_t ecc = TWAI_ECC_REG;
    const char *type[] = { "bit", "form", "stuff", "other" };
    printf("# last error: %s error, %s, segment 0x%02lX (raw 0x%02lX)\n",
           type[(ecc >> 6) & 0x3],
           (ecc & 0x20) ? "while receiving" : "while transmitting",
           (unsigned long)(ecc & 0x1F),
           (unsigned long)(ecc & 0xFF));
}

/* Set to 1 to also print every other frame on the bus as raw hex. */
#define LOG_ALL_FRAMES 0

/* Set to 1 to have the driver also log each alert itself (TWAI_ALERT_AND_LOG),
 * from its ISR, as it happens. Noisy and adds ISR latency; the alerts are
 * printed by this code anyway after each send. */
#define LOG_DRIVER_ALERTS 1

/* Every alert the driver can raise except RX_DATA, which fires on every
 * received frame and would bury the rest on a busy bus. */
#define TX_ALERTS (TWAI_ALERT_ALL & ~TWAI_ALERT_RX_DATA)

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

/* Print the name of every alert set in `alerts`, on one line. */
static void print_alerts(const char *tag, uint32_t alerts)
{
    static const struct { uint32_t bit; const char *name; } names[] = {
        { TWAI_ALERT_TX_IDLE,              "TX_IDLE" },
        { TWAI_ALERT_TX_SUCCESS,           "TX_SUCCESS" },
        { TWAI_ALERT_TX_FAILED,            "TX_FAILED" },
        { TWAI_ALERT_TX_RETRIED,           "TX_RETRIED" },
        { TWAI_ALERT_ARB_LOST,             "ARB_LOST" },
        { TWAI_ALERT_BUS_ERROR,            "BUS_ERROR" },
        { TWAI_ALERT_ABOVE_ERR_WARN,       "ABOVE_ERR_WARN" },
        { TWAI_ALERT_BELOW_ERR_WARN,       "BELOW_ERR_WARN" },
        { TWAI_ALERT_ERR_ACTIVE,           "ERR_ACTIVE" },
        { TWAI_ALERT_ERR_PASS,             "ERR_PASS" },
        { TWAI_ALERT_BUS_OFF,              "BUS_OFF" },
        { TWAI_ALERT_RECOVERY_IN_PROGRESS, "RECOVERY_IN_PROGRESS" },
        { TWAI_ALERT_BUS_RECOVERED,        "BUS_RECOVERED" },
        { TWAI_ALERT_RX_DATA,              "RX_DATA" },
        { TWAI_ALERT_RX_QUEUE_FULL,        "RX_QUEUE_FULL" },
        { TWAI_ALERT_RX_FIFO_OVERRUN,      "RX_FIFO_OVERRUN" },
        { TWAI_ALERT_PERIPH_RESET,         "PERIPH_RESET" },
    };
    printf("%12.6f %s alerts 0x%05lX:", now_s(), tag, (unsigned long)alerts);
    if (!alerts) {
        printf(" none");
    }
    for (size_t i = 0; i < sizeof(names) / sizeof(names[0]); i++) {
        if (alerts & names[i].bit) {
            printf(" %s", names[i].name);
        }
    }
    printf("\n");
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

static void print_status(void)
{
    static const char *state_names[] = { "STOPPED", "RUNNING", "BUS_OFF", "RECOVERING" };
    twai_status_info_t status;
    esp_err_t err = twai_get_status_info(&status);
    if (err != ESP_OK) {
        printf("# status: twai_get_status_info failed: %s\n", esp_err_to_name(err));
        return;
    }
    printf("# status: state=%s tx_err=%lu rx_err=%lu tx_queued=%lu rx_queued=%lu "
           "tx_failed=%lu arb_lost=%lu bus_err=%lu rx_missed=%lu rx_overrun=%lu\n",
           status.state < ARRAY_LEN(state_names) ? state_names[status.state] : "?",
           (unsigned long)status.tx_error_counter, (unsigned long)status.rx_error_counter,
           (unsigned long)status.msgs_to_tx, (unsigned long)status.msgs_to_rx,
           (unsigned long)status.tx_failed_count, (unsigned long)status.arb_lost_count,
           (unsigned long)status.bus_error_count, (unsigned long)status.rx_missed_count,
           (unsigned long)status.rx_overrun_count);
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

    printf("%12.6f TX %03lX  %d ", now_s(), (unsigned long)req.identifier, req.data_length_code);
    for (int i = 0; i < req.data_length_code; i++) {
        printf(" %02X", req.data[i]);
    }
    printf("\n");

    esp_err_t err = twai_transmit(&req, pdMS_TO_TICKS(REPLY_TIMEOUT_MS));
    if (err != ESP_OK) {
        /* ESP_ERR_TIMEOUT: TX queue full; ESP_FAIL: TX queue disabled and a
         * frame is still sending; ESP_ERR_INVALID_STATE: not running (e.g.
         * bus-off); ESP_ERR_NOT_SUPPORTED: listen-only mode. */
        printf("%12.6f TX twai_transmit failed: %s\n", now_s(), esp_err_to_name(err));
        print_status();
    }
    return err == ESP_OK;
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
    printf("# OBD-II reader at 500kbps, TX=GPIO%d RX=GPIO%d\n", TWAI_TX_GPIO, TWAI_RX_GPIO);

    uint32_t alert_mask = TX_ALERTS;
    if (LOG_DRIVER_ALERTS) {
        alert_mask |= TWAI_ALERT_AND_LOG;
    }
    err = twai_reconfigure_alerts(alert_mask, NULL);
    if (err != ESP_OK) {
        printf("# twai_reconfigure_alerts failed: %s\n", esp_err_to_name(err));
    }

    wait_for_bus_activity();

    int64_t last_status_us = esp_timer_get_time();
    int consecutive_fails = 0;

    while (1) {
        for (size_t i = 0; i < ARRAY_LEN(pids); i++) {
            const obd_pid_t *p = &pids[i];

            /* Clear alerts raised since the last send (e.g. while waiting for
             * bus activity) so the ones read below belong to this request. */
            uint32_t alerts = 0;
            if (twai_read_alerts(&alerts, 0) == ESP_OK && alerts) {
                print_alerts("pre-send", alerts);
            }

            if (!send_request(p->pid)) {
                printf("# could not queue request for %s\n", p->name);
            } else if (!wait_for_reply(p)) {
                printf("%12.6f %-15s no reply\n", now_s(), p->name);
            }

            /* The TX error counter only drops by one per successful send, so
             * judge each attempt by its alert rather than by the counter. */
            alerts = 0;
            esp_err_t alert_err = twai_read_alerts(&alerts, pdMS_TO_TICKS(50));
            if (alert_err == ESP_OK) {
                print_alerts("TX", alerts);
            } else {
                printf("%12.6f TX no alerts (%s)\n", now_s(), esp_err_to_name(alert_err));
            }
            if (alerts & (TWAI_ALERT_TX_FAILED | TWAI_ALERT_BUS_ERROR)) {
                print_last_error();
            }

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
