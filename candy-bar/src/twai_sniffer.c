/*
 * twai_sniffer.c
 *
 * Passive CAN sniffer for the XIAO ESP32-S3 + SN65HVD230 transceiver:
 *
 *   SN65HVD230 CTX -> GPIO4 (D3)
 *   SN65HVD230 CRX -> GPIO5 (D4)
 *   SN65HVD230 CANH/CANL -> OBD-II pins 6/14
 *   SN65HVD230 GND -> XIAO GND and OBD-II pin 5 (signal ground)
 *
 * Runs the controller in listen-only mode: it never transmits, not even
 * ACK bits or error frames, so it can't disturb the car's bus. Frames
 * are buffered in RAM and sent once a second as a binary dump: a "CNDY"
 * + uint32 count header followed by 16-byte capture_t entries
 * (little-endian).
 *
 * Dumps go out the bulk IN endpoint of a vendor-specific USB interface
 * (class 0xFF) on the S3's OTG peripheral; see usb_init(). Taking the
 * PHY disconnects the USB Serial/JTAG port, so to flash, hold BOOT while
 * plugging in or resetting to enter the ROM loader.
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/twai.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "tinyusb.h"
#include "tinyusb_default_config.h"

#define TWAI_TX_GPIO GPIO_NUM_4
#define TWAI_RX_GPIO GPIO_NUM_5

#define STATUS_PERIOD_MS 5000
#define DUMP_PERIOD_MS 1000

#define ARRAY_LEN(a) (sizeof(a) / sizeof((a)[0]))

/* pid.codes' test VID/PID, reserved for private development use. Needs a
 * real allocation before this goes to anyone else. Must match candy-man. */
#define USB_VID 0x1209
#define USB_PID 0x0001

#define USB_EP_OUT 0x01
#define USB_EP_IN 0x81

/* Give up on a dump if the host hasn't drained any of it for this long,
 * e.g. the device is plugged in but candy-man isn't running, so the TWAI
 * queue doesn't overflow while we wait. */
#define USB_WRITE_STALL_MS 50

/* Kept in internal SRAM (.bss) rather than PSRAM, which isn't enabled,
 * so an oversized buffer fails at link time rather than at runtime. */
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

/* Each dump is this header followed by `count` raw capture_t entries.
 * A dump abandoned partway (see usb_write()) leaves a truncated one in
 * the stream; a reader scans for the magic to find the next one. */
typedef struct {
    char magic[4];
    // TODO add base timestamp that all t_us values are relative to, so a reader can reconstruct the full 64-bit timestamp.
    uint32_t count;
} dump_header_t;

static const tusb_desc_device_t s_usb_device_desc = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    /* Class comes from the interface descriptor. */
    .bDeviceClass = 0,
    .bDeviceSubClass = 0,
    .bDeviceProtocol = 0,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = USB_VID,
    .idProduct = USB_PID,
    .bcdDevice = 0x0100,
    .iManufacturer = 1,
    .iProduct = 2,
    .iSerialNumber = 0,
    .bNumConfigurations = 1,
};

#define USB_CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_VENDOR_DESC_LEN)

static const uint8_t s_usb_config_desc[] = {
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, USB_CONFIG_TOTAL_LEN, 0, 100),
    /* Interface 0, no string, bulk OUT + bulk IN, 64-byte full-speed packets.
     * OUT is unused for now. */
    TUD_VENDOR_DESCRIPTOR(0, 0, USB_EP_OUT, USB_EP_IN, 64),
};

static const char *s_usb_strings[] = {
    (const char[]){ 0x09, 0x04 }, /* 0: English (US) */
    "candy",                      /* 1: manufacturer */
    "candy-bar",                  /* 2: product */
};

static esp_err_t usb_init(void)
{
    tinyusb_config_t cfg = TINYUSB_DEFAULT_CONFIG();
    cfg.descriptor.device = &s_usb_device_desc;
    cfg.descriptor.full_speed_config = s_usb_config_desc;
    cfg.descriptor.string = s_usb_strings;
    cfg.descriptor.string_count = ARRAY_LEN(s_usb_strings);
    return tinyusb_driver_install(&cfg);
}

/* Queues all of buf on the bulk IN endpoint. Returns false if the host
 * went away or stopped reading partway through. */
static bool usb_write(const void *buf, size_t len)
{
    const uint8_t *p = buf;
    int64_t last_progress_us = esp_timer_get_time();
    while (len > 0) {
        if (!tud_vendor_mounted()) {
            return false;
        }
        uint32_t n = tud_vendor_write(p, len);
        tud_vendor_write_flush();
        if (n > 0) {
            p += n;
            len -= n;
            last_progress_us = esp_timer_get_time();
        } else if (esp_timer_get_time() - last_progress_us >= USB_WRITE_STALL_MS * 1000LL) {
            return false;
        } else {
            vTaskDelay(1);
        }
    }
    return true;
}

/* If nobody's listening the dump is dropped, not held for later. */
static void dump_capture(void)
{
    dump_header_t hdr = { .magic = { 'C', 'N', 'D', 'Y' }, .count = s_capture_len };
    if (usb_write(&hdr, sizeof(hdr))) {
        usb_write(s_capture, s_capture_len * sizeof(capture_t));
    }
    s_capture_len = 0;
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
    esp_err_t err = usb_init();
    if (err != ESP_OK) {
        printf("# tinyusb_driver_install failed: %s\n", esp_err_to_name(err));
        return;
    }

    twai_general_config_t g_config =
        TWAI_GENERAL_CONFIG_DEFAULT(TWAI_TX_GPIO, TWAI_RX_GPIO, TWAI_MODE_LISTEN_ONLY);
    /* The default 5-frame queue overflows within milliseconds on a busy bus. */
    g_config.rx_queue_len = 128;
    /* No alerts: the driver would otherwise log some of them from its ISR. */
    g_config.alerts_enabled = TWAI_ALERT_NONE;
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    err = twai_driver_install(&g_config, &t_config, &f_config);
    if (err != ESP_OK) {
        printf("# twai_driver_install failed: %s\n", esp_err_to_name(err));
        return;
    }
    err = twai_start();
    if (err != ESP_OK) {
        printf("# twai_start failed: %s\n", esp_err_to_name(err));
        return;
    }

    int64_t last_status_us = esp_timer_get_time();
    int64_t last_dump_us = last_status_us;
    while (1) {
        twai_message_t msg;
        if (twai_receive(&msg, pdMS_TO_TICKS(DUMP_PERIOD_MS)) == ESP_OK) {
            capture(&msg);
        }
        if (esp_timer_get_time() - last_dump_us >= DUMP_PERIOD_MS * 1000LL) {
            last_dump_us = esp_timer_get_time();
            dump_capture();
        }
        if (esp_timer_get_time() - last_status_us >= STATUS_PERIOD_MS * 1000LL) {
            last_status_us = esp_timer_get_time();
            // print_status();
        }
    }
}
