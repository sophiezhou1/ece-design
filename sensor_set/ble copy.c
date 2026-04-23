#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_err.h"

#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_adc/adc_continuous.h"
#include "hal/adc_types.h"
#include "hal/spi_types.h"
#include "soc/gpio_num.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"


static const char *TAG = "sensor_ble";

#define DEVICE_NAME             "sensor_set"
#define TX_PERIOD_MS            1000
#define SESSION_DURATION_MS     (60 * 60 * 1000)

#define PIN_SCK                 12
#define PIN_MISO                13
#define PIN_MOSI                11

#define PIN_MAX_CS              10

#define PIN_LCD_CS              14
#define PIN_LCD_DC              15
#define PIN_LCD_RST             16
#define PIN_LCD_BL              17

#define ADC_INPUT_GPIO          GPIO_NUM_1

#define LCD_W                   240
#define LCD_H                   320

static spi_device_handle_t g_lcddev = NULL;
static bool g_spi_bus_ready = false;

static uint32_t g_sample_count = 0;
static int64_t g_boot_us = 0;

static const ble_uuid128_t SENSOR_SERVICE_UUID =
    BLE_UUID128_INIT(0x78, 0x56, 0x34, 0x12, 0x9A, 0xBC, 0xDE, 0xF0,
                     0x12, 0x34, 0x56, 0x78, 0x00, 0x00, 0xEE, 0x01);

static const ble_uuid128_t SENSOR_DATA_CHAR_UUID =
    BLE_UUID128_INIT(0x78, 0x56, 0x34, 0x12, 0x9A, 0xBC, 0xDE, 0xF0,
                     0x12, 0x34, 0x56, 0x78, 0x01, 0x00, 0xEE, 0x01);
static const ble_uuid128_t ADV_SERVICE_UUIDS[] = {
    BLE_UUID128_INIT(0x78, 0x56, 0x34, 0x12, 0x9A, 0xBC, 0xDE, 0xF0,
                     0x12, 0x34, 0x56, 0x78, 0x00, 0x00, 0xEE, 0x01)
};

#pragma pack(push, 1)
typedef struct {
    uint32_t seq;
    uint64_t timestamp_us;
    float thermocouple_c;
    float food_probe_c;
    uint8_t control_flags;
    uint8_t tc_fault_flags;
    uint8_t reserved[2];
} sensor_packet_t;
#pragma pack(pop)

static uint16_t g_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t g_data_char_handle = 0;
static bool g_notify_enabled = false;
static uint32_t g_seq = 0;
static uint8_t g_own_addr_type = BLE_OWN_ADDR_PUBLIC;
static portMUX_TYPE g_pkt_lock = portMUX_INITIALIZER_UNLOCKED;
static sensor_packet_t g_latest_pkt = {0};
static spi_device_handle_t g_maxdev = NULL;

static void ble_app_advertise(void);

// static esp_err_t max31855_init(void)
// {
//     spi_bus_config_t buscfg = {
//         .mosi_io_num = -1,
//         .miso_io_num = PIN_MISO,
//         .sclk_io_num = PIN_SCK,
//         .quadwp_io_num = -1,
//         .quadhd_io_num = -1,
//         .max_transfer_sz = 4
//     };

//     spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);

//     spi_device_interface_config_t devcfg = {
//         .clock_speed_hz = 1000000,
//         .mode = 0,
//         .spics_io_num = PIN_CS,
//         .queue_size = 1,
//         .flags = SPI_DEVICE_HALFDUPLEX
//     };
//     return spi_bus_add_device(SPI2_HOST, &devcfg, &g_maxdev);
// }

static esp_err_t max31855_read_raw(uint32_t *raw_out)
{
    uint8_t rx[4] = {0};
    spi_transaction_t t = {0};
    t.length = 0;
    t.rxlength = 32;
    t.rx_buffer = rx;

    esp_err_t err = spi_device_transmit(g_maxdev, &t);
    if (err != ESP_OK) {
        return err;
    }

    *raw_out = ((uint32_t)rx[0] << 24) |
               ((uint32_t)rx[1] << 16) |
               ((uint32_t)rx[2] << 8)  |
               ((uint32_t)rx[3]);
    return ESP_OK;
}

static float max31855_tctemp_fault(uint32_t raw, bool *fault_out)
{
    bool fault = (raw >> 16) & 1u;
    if (fault_out) {
        *fault_out = fault;
    }

    int32_t t14 = (int32_t)((raw >> 18) & 0x3FFFu);
    if (t14 & 0x2000) {
        t14 |= ~0x3FFF;
    }
    return (float)t14 * 0.25f;
}

static float max31855_internal(uint32_t raw)
{
    int32_t intern_temp = (int32_t)((raw >> 4) & 0x0FFFu);
    if (intern_temp & 0x0800) {
        intern_temp |= ~0x0FFF;
    }
    return (float)intern_temp * 0.0625f;
}

static sensor_packet_t collect_sensor_packet(void)
{
    sensor_packet_t pkt;
    taskENTER_CRITICAL(&g_pkt_lock);
    pkt = g_latest_pkt;
    taskEXIT_CRITICAL(&g_pkt_lock);
    pkt.seq = g_seq++;
    return pkt;
}

static esp_err_t spi_bus_setup_once(void)
{
    if (g_spi_bus_ready) {
        return ESP_OK;
    }

    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = PIN_MISO,
        .sclk_io_num = PIN_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096
    };

    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
    g_spi_bus_ready = true;
    return ESP_OK;
}

static esp_err_t max31855_init(void)
{
    ESP_ERROR_CHECK(spi_bus_setup_once());

    if (g_maxdev != NULL) {
        return ESP_OK;
    }

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1000000,
        .mode = 0,
        .spics_io_num = PIN_MAX_CS,
        .queue_size = 1,
        .flags = SPI_DEVICE_HALFDUPLEX
    };

    return spi_bus_add_device(SPI2_HOST, &devcfg, &g_maxdev);
}

static esp_err_t lcd_add_device(void)
{
    ESP_ERROR_CHECK(spi_bus_setup_once());

    if (g_lcddev != NULL) {
        return ESP_OK;
    }

    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask =
            (1ULL << PIN_LCD_DC) |
            (1ULL << PIN_LCD_RST) |
            (1ULL << PIN_LCD_BL),
        .pull_down_en = 0,
        .pull_up_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 26000000,
        .mode = 0,
        .spics_io_num = PIN_LCD_CS,
        .queue_size = 1,
        .flags = SPI_DEVICE_HALFDUPLEX
    };

    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &g_lcddev));
    return ESP_OK;
}

static void lcd_send(bool is_data, const void *data, size_t len_bytes)
{
    if (len_bytes == 0) return;

    gpio_set_level(PIN_LCD_DC, is_data ? 1 : 0);

    spi_transaction_t t = {0};
    t.length = len_bytes * 8;
    t.tx_buffer = data;
    ESP_ERROR_CHECK(spi_device_transmit(g_lcddev, &t));
}

static void lcd_cmd(uint8_t cmd)
{
    lcd_send(false, &cmd, 1);
}

static void lcd_data(const void *data, size_t len)
{
    lcd_send(true, data, len);
}

static void lcd_reset(void)
{
    gpio_set_level(PIN_LCD_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(PIN_LCD_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(120));
}

static void lcd_set_addr_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    uint8_t data[4];

    lcd_cmd(0x2A); // CASET
    data[0] = x0 >> 8; data[1] = x0 & 0xFF;
    data[2] = x1 >> 8; data[3] = x1 & 0xFF;
    lcd_data(data, 4);

    lcd_cmd(0x2B); // PASET
    data[0] = y0 >> 8; data[1] = y0 & 0xFF;
    data[2] = y1 >> 8; data[3] = y1 & 0xFF;
    lcd_data(data, 4);

    lcd_cmd(0x2C); // RAMWR
}

static void lcd_init(void)
{
    ESP_ERROR_CHECK(lcd_add_device());
    lcd_reset();

    uint8_t data[16];

    lcd_cmd(0x01); // SWRESET
    vTaskDelay(pdMS_TO_TICKS(120));

    lcd_cmd(0x28); // display off

    data[0]=0x03; data[1]=0x80; data[2]=0x02;
    lcd_cmd(0xEF); lcd_data(data, 3);

    data[0]=0x00; data[1]=0xC1; data[2]=0x30;
    lcd_cmd(0xCF); lcd_data(data, 3);

    data[0]=0x64; data[1]=0x03; data[2]=0x12; data[3]=0x81;
    lcd_cmd(0xED); lcd_data(data, 4);

    data[0]=0x85; data[1]=0x00; data[2]=0x78;
    lcd_cmd(0xE8); lcd_data(data, 3);

    data[0]=0x39; data[1]=0x2C; data[2]=0x00; data[3]=0x34; data[4]=0x02;
    lcd_cmd(0xCB); lcd_data(data, 5);

    data[0]=0x20;
    lcd_cmd(0xF7); lcd_data(data, 1);

    data[0]=0x00; data[1]=0x00;
    lcd_cmd(0xEA); lcd_data(data, 2);

    data[0]=0x23;
    lcd_cmd(0xC0); lcd_data(data, 1); // power control 1

    data[0]=0x10;
    lcd_cmd(0xC1); lcd_data(data, 1); // power control 2

    data[0]=0x3E; data[1]=0x28;
    lcd_cmd(0xC5); lcd_data(data, 2); // VCOM

    data[0]=0x86;
    lcd_cmd(0xC7); lcd_data(data, 1);

    data[0]=0x48;                 // rotation
    lcd_cmd(0x36); lcd_data(data, 1);

    data[0]=0x55;                 // RGB565
    lcd_cmd(0x3A); lcd_data(data, 1);

    data[0]=0x00; data[1]=0x18;
    lcd_cmd(0xB1); lcd_data(data, 2);

    data[0]=0x08; data[1]=0x82; data[2]=0x27;
    lcd_cmd(0xB6); lcd_data(data, 3);

    data[0]=0x00;
    lcd_cmd(0xF2); lcd_data(data, 1);

    data[0]=0x01;
    lcd_cmd(0x26); lcd_data(data, 1);

    uint8_t gamma_pos[15] = {
        0x0F,0x31,0x2B,0x0C,0x0E,0x08,0x4E,0xF1,0x37,0x07,0x10,0x03,0x0E,0x09,0x00
    };
    lcd_cmd(0xE0); lcd_data(gamma_pos, 15);

    uint8_t gamma_neg[15] = {
        0x00,0x0E,0x14,0x03,0x11,0x07,0x31,0xC1,0x48,0x08,0x0F,0x0C,0x31,0x36,0x0F
    };
    lcd_cmd(0xE1); lcd_data(gamma_neg, 15);

    lcd_cmd(0x11); // sleep out
    vTaskDelay(pdMS_TO_TICKS(120));

    lcd_cmd(0x29); // display on
    vTaskDelay(pdMS_TO_TICKS(20));

    gpio_set_level(PIN_LCD_BL, 1);
}

#define RGB565(r,g,b)   (uint16_t)((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3))
#define C_BLACK         RGB565(0,0,0)
#define C_WHITE         RGB565(255,255,255)
#define C_RED           RGB565(255,0,0)
#define C_GREEN         RGB565(0,255,0)
#define C_BLUE          RGB565(0,100,255)
#define C_YELLOW        RGB565(255,220,0)
#define C_ORANGE        RGB565(255,140,0)
#define C_GRAY          RGB565(70,70,70)
#define C_DARK          RGB565(15,15,25)
#define C_CYAN          RGB565(0,220,220)

static void lcd_fill_rect(int x, int y, int w, int h, uint16_t color)
{
    if (w <= 0 || h <= 0) return;
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x + w > LCD_W) w = LCD_W - x;
    if (y + h > LCD_H) h = LCD_H - y;
    if (w <= 0 || h <= 0) return;

    lcd_set_addr_window(x, y, x + w - 1, y + h - 1);

    uint8_t buf[128];
    for (int i = 0; i < 64; i++) {
        buf[2 * i] = color >> 8;
        buf[2 * i + 1] = color & 0xFF;
    }

    int pixels = w * h;
    while (pixels > 0) {
        int chunk = (pixels > 64) ? 64 : pixels;
        lcd_data(buf, chunk * 2);
        pixels -= chunk;
    }
}

static void lcd_fill_screen(uint16_t color)
{
    lcd_fill_rect(0, 0, LCD_W, LCD_H, color);
}

static void lcd_draw_bar(int x, int y, int w, int h, float frac, uint16_t fg, uint16_t bg)
{
    if (frac < 0.0f) frac = 0.0f;
    if (frac > 1.0f) frac = 1.0f;
    lcd_fill_rect(x, y, w, h, bg);
    lcd_fill_rect(x, y, (int)(w * frac), h, fg);
}

static void glyph3x5(char c, uint8_t out[5])
{
    memset(out, 0, 5);

    switch (c) {
    case '0': out[0]=7; out[1]=5; out[2]=5; out[3]=5; out[4]=7; break;
    case '1': out[0]=2; out[1]=6; out[2]=2; out[3]=2; out[4]=7; break;
    case '2': out[0]=7; out[1]=1; out[2]=7; out[3]=4; out[4]=7; break;
    case '3': out[0]=7; out[1]=1; out[2]=7; out[3]=1; out[4]=7; break;
    case '4': out[0]=5; out[1]=5; out[2]=7; out[3]=1; out[4]=1; break;
    case '5': out[0]=7; out[1]=4; out[2]=7; out[3]=1; out[4]=7; break;
    case '6': out[0]=7; out[1]=4; out[2]=7; out[3]=5; out[4]=7; break;
    case '7': out[0]=7; out[1]=1; out[2]=1; out[3]=1; out[4]=1; break;
    case '8': out[0]=7; out[1]=5; out[2]=7; out[3]=5; out[4]=7; break;
    case '9': out[0]=7; out[1]=5; out[2]=7; out[3]=1; out[4]=7; break;

    case 'A': out[0]=2; out[1]=5; out[2]=7; out[3]=5; out[4]=5; break;
    case 'B': out[0]=6; out[1]=5; out[2]=6; out[3]=5; out[4]=6; break;
    case 'C': out[0]=3; out[1]=4; out[2]=4; out[3]=4; out[4]=3; break;
    case 'D': out[0]=6; out[1]=5; out[2]=5; out[3]=5; out[4]=6; break;
    case 'E': out[0]=7; out[1]=4; out[2]=6; out[3]=4; out[4]=7; break;
    case 'F': out[0]=7; out[1]=4; out[2]=6; out[3]=4; out[4]=4; break;
    case 'G': out[0]=3; out[1]=4; out[2]=5; out[3]=5; out[4]=3; break;
    case 'H': out[0]=5; out[1]=5; out[2]=7; out[3]=5; out[4]=5; break;
    case 'I': out[0]=7; out[1]=2; out[2]=2; out[3]=2; out[4]=7; break;
    case 'K': out[0]=5; out[1]=5; out[2]=6; out[3]=5; out[4]=5; break;
    case 'L': out[0]=4; out[1]=4; out[2]=4; out[3]=4; out[4]=7; break;
    case 'M': out[0]=5; out[1]=7; out[2]=7; out[3]=5; out[4]=5; break;
    case 'N': out[0]=5; out[1]=7; out[2]=7; out[3]=7; out[4]=5; break;
    case 'O': out[0]=2; out[1]=5; out[2]=5; out[3]=5; out[4]=2; break;
    case 'P': out[0]=6; out[1]=5; out[2]=6; out[3]=4; out[4]=4; break;
    case 'Q': out[0]=2; out[1]=5; out[2]=5; out[3]=7; out[4]=3; break;
    case 'R': out[0]=6; out[1]=5; out[2]=6; out[3]=5; out[4]=5; break;
    case 'S': out[0]=3; out[1]=4; out[2]=2; out[3]=1; out[4]=6; break;
    case 'T': out[0]=7; out[1]=2; out[2]=2; out[3]=2; out[4]=2; break;
    case 'U': out[0]=5; out[1]=5; out[2]=5; out[3]=5; out[4]=7; break;
    case 'V': out[0]=5; out[1]=5; out[2]=5; out[3]=5; out[4]=2; break;
    case 'W': out[0]=5; out[1]=5; out[2]=7; out[3]=7; out[4]=5; break;
    case 'Y': out[0]=5; out[1]=5; out[2]=2; out[3]=2; out[4]=2; break;

    case ':': out[0]=0; out[1]=2; out[2]=0; out[3]=2; out[4]=0; break;
    case '.': out[0]=0; out[1]=0; out[2]=0; out[3]=0; out[4]=2; break;
    case '-': out[0]=0; out[1]=0; out[2]=7; out[3]=0; out[4]=0; break;
    case '/': out[0]=1; out[1]=1; out[2]=2; out[3]=4; out[4]=4; break;
    case ' ': default: break;
    }
}

static void lcd_draw_char(int x, int y, char c, uint16_t fg, uint16_t bg, int scale)
{
    uint8_t rows[5];
    glyph3x5(c, rows);

    for (int ry = 0; ry < 5; ry++) {
        for (int rx = 0; rx < 3; rx++) {
            uint16_t color = (rows[ry] & (1 << (2 - rx))) ? fg : bg;
            lcd_fill_rect(x + rx * scale, y + ry * scale, scale, scale, color);
        }
    }
}

static void lcd_draw_text(int x, int y, const char *s, uint16_t fg, uint16_t bg, int scale)
{
    while (*s) {
        lcd_draw_char(x, y, *s, fg, bg, scale);
        x += 4 * scale;
        s++;
    }
}

static void lcd_render_screen(sensor_packet_t pkt)
{
    char buf[64];

    bool tc_fault = (pkt.control_flags & 0x01u) != 0;
    bool adc_valid = (pkt.control_flags & 0x02u) != 0;
    bool connected = (g_conn_handle != BLE_HS_CONN_HANDLE_NONE);
    bool notifying = g_notify_enabled;

    uint16_t banner = C_BLUE;
    const char *state = "ADV";

    if (tc_fault) {
        banner = C_RED;
        state = "FAULT";
    } else if (notifying) {
        banner = C_GREEN;
        state = "NOTIFY";
    } else if (connected) {
        banner = C_CYAN;
        state = "CONN";
    }

    lcd_fill_screen(C_DARK);

    // Header
    lcd_fill_rect(0, 0, LCD_W, 36, banner);
    lcd_draw_text(8, 8, "FOOD PROBE", C_WHITE, banner, 3);
    lcd_draw_text(180, 8, state, C_BLACK, banner, 2);

    // TC block
    lcd_draw_text(10, 50, "TC", C_YELLOW, C_DARK, 3);
    snprintf(buf, sizeof(buf), "%.1f", pkt.thermocouple_c);
    lcd_draw_text(10, 78, buf, C_WHITE, C_DARK, 5);
    lcd_draw_text(170, 92, "C", C_WHITE, C_DARK, 4);

    // ADC raw
    lcd_draw_text(10, 150, "ADC", C_ORANGE, C_DARK, 3);
    snprintf(buf, sizeof(buf), "%.0f", pkt.food_probe_c);
    lcd_draw_text(80, 150, buf, C_WHITE, C_DARK, 3);

    float adc_frac = pkt.food_probe_c / 4095.0f;
    lcd_draw_bar(10, 182, 220, 16, adc_frac, C_ORANGE, C_GRAY);

    // BLE
    lcd_draw_text(10, 210, "BLE", C_CYAN, C_DARK, 2);
    if (notifying) {
        lcd_draw_text(70, 210, "NOTIFY", C_WHITE, C_DARK, 2);
    } else if (connected) {
        lcd_draw_text(70, 210, "CONN", C_WHITE, C_DARK, 2);
    } else {
        lcd_draw_text(70, 210, "ADV", C_WHITE, C_DARK, 2);
    }

    // Flags
    lcd_draw_text(10, 235, "FLAGS", C_YELLOW, C_DARK, 2);
    snprintf(buf, sizeof(buf), "0X%02X", pkt.tc_fault_flags);
    lcd_draw_text(90, 235, buf, C_WHITE, C_DARK, 2);

    // Sample count
    lcd_draw_text(10, 260, "SEQ", C_YELLOW, C_DARK, 2);
    snprintf(buf, sizeof(buf), "%lu", (unsigned long)g_sample_count);
    lcd_draw_text(60, 260, buf, C_WHITE, C_DARK, 2);

    // Uptime
    lcd_draw_text(10, 285, "UP", C_YELLOW, C_DARK, 2);
    snprintf(buf, sizeof(buf), "%lus", (unsigned long)((esp_timer_get_time() - g_boot_us) / 1000000ULL));
    lcd_draw_text(50, 285, buf, C_WHITE, C_DARK, 2);

    // Fault / ADC validity indicators
    lcd_fill_rect(180, 228, 18, 18, tc_fault ? C_RED : C_GREEN);
    lcd_fill_rect(210, 228, 18, 18, adc_valid ? C_GREEN : C_RED);
}

static void lcd_task(void *param)
{
    (void)param;

    lcd_init();
    lcd_fill_screen(C_BLACK);
    lcd_draw_text(40, 120, "BOOTING", C_WHITE, C_BLACK, 4);

    while (1) {
        sensor_packet_t pkt;
        taskENTER_CRITICAL(&g_pkt_lock);
        pkt = g_latest_pkt;
        taskEXIT_CRITICAL(&g_pkt_lock);

        lcd_render_screen(pkt);
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

static void sensor_task(void *param)
{
    (void)param;
    setvbuf(stdout, NULL, _IONBF, 0);

    adc_unit_t unit;
    adc_channel_t channel;
    ESP_ERROR_CHECK(adc_continuous_io_to_channel(ADC_INPUT_GPIO, &unit, &channel));

    adc_continuous_handle_t adc_handle = NULL;
    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 4096,
        .conv_frame_size = 1024,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &adc_handle));

    adc_digi_pattern_config_t pattern = {
        .atten = ADC_ATTEN_DB_12,
        .bit_width = ADC_BITWIDTH_12,
        .channel = channel,
        .unit = unit,
    };

    adc_continuous_config_t cfg = {
        .sample_freq_hz = 2000,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
        .pattern_num = 1,
        .adc_pattern = &pattern,
    };
    ESP_ERROR_CHECK(adc_continuous_config(adc_handle, &cfg));
    ESP_ERROR_CHECK(adc_continuous_start(adc_handle));

    ESP_ERROR_CHECK(max31855_init());

    uint8_t result[1024];
    static adc_continuous_data_t parsed_data[1024 / SOC_ADC_DIGI_RESULT_BYTES];
    const uint32_t DECIM_N = 20;
    uint32_t decim = 0;

    while (1) {
        uint32_t ret_num = 0;
        esp_err_t ret = adc_continuous_read(adc_handle, result, sizeof(result), &ret_num, 1000);
        if (ret != ESP_OK || ret_num == 0) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        uint32_t num_parsed_samples = 0;
        ret = adc_continuous_parse_data(adc_handle, result, ret_num, parsed_data, &num_parsed_samples);
        if (ret != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        uint32_t raw_tc = 0;
        if (max31855_read_raw(&raw_tc) != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        bool tc_fault = false;
        float tc = max31855_tctemp_fault(raw_tc, &tc_fault);
        float tc_internal = max31855_internal(raw_tc);
        uint8_t tc_fault_flags = (uint8_t)(raw_tc & 0x07u); // OC, SCG, SCV

        for (uint32_t i = 0; i < num_parsed_samples; i++) {
            if (!parsed_data[i].valid) {
                continue;
            }
            if ((++decim % DECIM_N) != 0) {
                continue;
            }

            sensor_packet_t next = {
                .seq = 0, // assigned on read/notify
                .timestamp_us = (uint64_t)esp_timer_get_time(),
                .thermocouple_c = tc,
                // Original file publishes raw ADC value; keep same source in this field.
                .food_probe_c = (float)parsed_data[i].raw_data,
                // bit0: thermocouple fault present, bit1: adc sample valid
                .control_flags = (uint8_t)((tc_fault ? 1u : 0u) | 0x02u),
                .tc_fault_flags = tc_fault_flags,
                .reserved = {0, 0},
            };

            taskENTER_CRITICAL(&g_pkt_lock);
            g_latest_pkt = next;
            g_sample_count++;
            taskEXIT_CRITICAL(&g_pkt_lock);

            ESP_LOGI(TAG,
                     "%" PRIu64 ",adc%.0f raw_tc=0x%08" PRIX32 " tc=%.2fC intern=%.2fC fault=%d flags=0x%02X",
                     next.timestamp_us, next.food_probe_c, raw_tc, tc, tc_internal, tc_fault, tc_fault_flags);
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

static int gatt_svr_chr_access_cb(uint16_t conn_handle,
                                  uint16_t attr_handle,
                                  struct ble_gatt_access_ctxt *ctxt,
                                  void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;

    sensor_packet_t pkt = collect_sensor_packet();
    int rc = os_mbuf_append(ctxt->om, &pkt, sizeof(pkt));
    return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &SENSOR_SERVICE_UUID.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &SENSOR_DATA_CHAR_UUID.u,
                .access_cb = gatt_svr_chr_access_cb,
                .val_handle = &g_data_char_handle,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
            },
            {0}
        },
    },
    {0}
};

static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            g_conn_handle = event->connect.conn_handle;
            ESP_LOGI(TAG, "Central connected, handle=%d", g_conn_handle);
        } else {
            ble_app_advertise();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        g_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        g_notify_enabled = false;
        ble_app_advertise();
        return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == g_data_char_handle) {
            g_notify_enabled = event->subscribe.cur_notify;
        }
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        ble_app_advertise();
        return 0;

    default:
        return 0;
    }
}

static void ble_app_advertise(void)
{
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields adv_fields;
    struct ble_hs_adv_fields rsp_fields;

    memset(&adv_fields, 0, sizeof(adv_fields));
    adv_fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    adv_fields.uuids128 = ADV_SERVICE_UUIDS;
    adv_fields.num_uuids128 = 1;
    adv_fields.uuids128_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&adv_fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_set_fields failed: %d", rc);
        return;
    }

    memset(&rsp_fields, 0, sizeof(rsp_fields));
    rsp_fields.name = (const uint8_t *)DEVICE_NAME;
    rsp_fields.name_len = strlen(DEVICE_NAME);
    rsp_fields.name_is_complete = 1;

    rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_rsp_set_fields failed: %d", rc);
        return;
    }

    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(g_own_addr_type, NULL,
                           BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_start failed: %d", rc);
    }
}

static void ble_on_sync(void)
{
    int rc = ble_hs_id_infer_auto(0, &g_own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_hs_id_infer_auto failed: %d", rc);
        return;
    }
    ble_app_advertise();
}

static void host_task(void *param)
{
    (void)param;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

static void notify_task(void *param)
{
    (void)param;

    const int64_t start_us = esp_timer_get_time();
    while ((esp_timer_get_time() - start_us) < ((int64_t)SESSION_DURATION_MS * 1000LL)) {
        if (g_conn_handle != BLE_HS_CONN_HANDLE_NONE && g_notify_enabled) {
            sensor_packet_t pkt = collect_sensor_packet();
            struct os_mbuf *om = ble_hs_mbuf_from_flat(&pkt, sizeof(pkt));
            int rc = ble_gattc_notify_custom(g_conn_handle, g_data_char_handle, om);
            if (rc != 0) {
                ESP_LOGW(TAG, "Notify failed: %d", rc);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(TX_PERIOD_MS));
    }

    ESP_LOGI(TAG, "60-minute telemetry window complete.");
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    nimble_port_init();

    ble_svc_gap_init();
    ble_svc_gatt_init();

    int rc = ble_gatts_count_cfg(gatt_svcs);
    ESP_ERROR_CHECK(rc == 0 ? ESP_OK : ESP_FAIL);
    rc = ble_gatts_add_svcs(gatt_svcs);
    ESP_ERROR_CHECK(rc == 0 ? ESP_OK : ESP_FAIL);

    ble_svc_gap_device_name_set(DEVICE_NAME);
    ble_hs_cfg.sync_cb = ble_on_sync;

    g_boot_us = esp_timer_get_time();

    xTaskCreate(sensor_task, "sensor_task", 8192, NULL, 6, NULL);
    nimble_port_freertos_init(host_task);
    xTaskCreate(notify_task, "notify_task", 4096, NULL, 5, NULL);
    xTaskCreate(lcd_task, "lcd_task", 8192, NULL, 4, NULL);
}
