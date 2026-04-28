#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <inttypes.h>
#include <math.h>

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

/* Shared SPI bus pins from your pinout */
// #define PIN_SCK                 12
// #define PIN_MISO                13
// #define PIN_MOSI                11
#define PIN_SCK                 12
#define PIN_MISO                10
#define PIN_MOSI                11

/* MAX31855 */
#define PIN_MAX_CS              9

/* LCD */
#define PIN_LCD_CS              16
#define PIN_LCD_DC              7
#define PIN_LCD_RST             6
#define PIN_LCD_BL              21
#define LCD_W                   320
#define LCD_H                   240
#define PAN_MAX_TEMP_C          35.0f

#define ADC_INPUT_GPIO          4

#define STEPPER_PHASE_DELAY_MS          4
#define STEPPER_ACTIVATE_STEPS          100
#define STEPPER_TRIM_LEFT_STEPS         100
#define STEPPER_SHUTOFF_LEFT_STEPS      3000
#define OVERTEMP_TRIM_DELAY_MS          8000
#define OVERTEMP_SHUTOFF_DELAY_MS       30000
#define OVERTEMP_MIN_DROP_C             2.0f

#define COLOR_BLACK             0x0000
#define COLOR_WHITE             0xFFFF
#define COLOR_RED               0xF800
#define COLOR_GREEN             0x07E0
#define COLOR_BLUE              0x001F
#define COLOR_YELLOW            0xFFE0
#define COLOR_CYAN              0x07FF
#define COLOR_DARKGREY          0x7BEF

#define CTRL_FLAG_TC_FAULT          0x01u
#define CTRL_FLAG_SAMPLE_VALID      0x02u
#define CTRL_FLAG_PROBE_DISCONNECT  0x04u
#define CTRL_FLAG_TC_DISCONNECT     0x08u

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

static const gpio_num_t g_stepper_pins[4] = {
    GPIO_NUM_15,
    GPIO_NUM_16,
    GPIO_NUM_17,
    GPIO_NUM_18,
};

static const uint8_t g_step_sequence[4][4] = {
    {1, 0, 0, 0},
    {0, 1, 0, 0},
    {0, 0, 1, 0},
    {0, 0, 0, 1},
};

/* SPI devices */
static spi_device_handle_t g_maxdev = NULL;
static spi_device_handle_t g_lcddev = NULL;
static bool g_spi_bus_ready = false;
static bool g_stepper_ready = false;
static int g_stepper_phase = 0;

static void ble_app_advertise(void);

static void stepper_init_once(void) {
    if (g_stepper_ready) {
        return;
    }

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << 15) | (1ULL << 16) | (1ULL << 17) | (1ULL << 18),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    for (int p = 0; p < 4; p++) {
        gpio_set_level(g_stepper_pins[p], g_step_sequence[g_stepper_phase][p]);
    }
    g_stepper_ready = true;
}

static void stepper_handle_overtemp(float pan_temp_c) {
    bool overtemp = pan_temp_c > PAN_MAX_TEMP_C;
    int step_counter = 0;
    if (overtemp) {
        stepper_init_once();
        while (step_counter < 1000) {
            for (int s = 0; s < 4; s++) {
                gpio_set_level(15, g_step_sequence[s][0]);
                step_counter++;
                gpio_set_level(16, g_step_sequence[s][1]);
                step_counter++;
                gpio_set_level(17, g_step_sequence[s][2]);
                step_counter++;
                gpio_set_level(18, g_step_sequence[s][3]);
                step_counter++;
                vTaskDelay(pdMS_TO_TICKS(20));
                printf("%d ", step_counter);
            }
        }
    }
}

/* --------------------------- LCD helpers --------------------------- */

static inline void lcd_dc_cmd(void) {
    gpio_set_level(PIN_LCD_DC, 0);
}

static inline void lcd_dc_data(void) {
    gpio_set_level(PIN_LCD_DC, 1);
}

static void lcd_send_cmd(uint8_t cmd) {
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &cmd,
    };
    lcd_dc_cmd();
    ESP_ERROR_CHECK(spi_device_transmit(g_lcddev, &t));
}

static void lcd_send_data(const void *data, int len_bytes) {
    if (len_bytes <= 0) return;
    spi_transaction_t t = {
        .length = len_bytes * 8,
        .tx_buffer = data,
    };
    lcd_dc_data();
    ESP_ERROR_CHECK(spi_device_transmit(g_lcddev, &t));
}

static void lcd_send_u8(uint8_t data) {
    lcd_send_data(&data, 1);
}

static void lcd_send_u16_be(uint16_t value) {
    uint8_t buf[2] = { (uint8_t)(value >> 8), (uint8_t)(value & 0xFF) };
    lcd_send_data(buf, 2);
}

static void lcd_reset_hw(void) {
    gpio_set_level(PIN_LCD_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(PIN_LCD_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(120));
}

static void lcd_set_addr_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    lcd_send_cmd(0x2A);
    lcd_send_u16_be(x0);
    lcd_send_u16_be(x1);

    lcd_send_cmd(0x2B);
    lcd_send_u16_be(y0);
    lcd_send_u16_be(y1);

    lcd_send_cmd(0x2C);
}

static void lcd_fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    if (x >= LCD_W || y >= LCD_H) return;
    if (x + w > LCD_W) w = LCD_W - x;
    if (y + h > LCD_H) h = LCD_H - y;
    if (w == 0 || h == 0) return;

    lcd_set_addr_window(x, y, x + w - 1, y + h - 1);

    const int chunk_pixels = 128;
    uint16_t linebuf[chunk_pixels];
    for (int i = 0; i < chunk_pixels; i++) {
        linebuf[i] = (uint16_t)((color << 8) | (color >> 8)); // byte-swapped for SPI
    }

    int total = w * h;
    while (total > 0) {
        int n = (total > chunk_pixels) ? chunk_pixels : total;
        lcd_send_data(linebuf, n * 2);
        total -= n;
    }
}

static void lcd_draw_hseg(int x, int y, int len, int thick, uint16_t color) {
    lcd_fill_rect(x, y, len, thick, color);
}

static void lcd_draw_vseg(int x, int y, int thick, int len, uint16_t color) {
    lcd_fill_rect(x, y, thick, len, color);
}

static void lcd_draw_colon7(int x, int y, int scale, uint16_t color, uint16_t bg) {
    int s = 4 * scale;
    lcd_fill_rect(x, y, 6 * scale, 32 * scale, bg);
    lcd_fill_rect(x, y + 8 * scale, s, s, color);
    lcd_fill_rect(x, y + 20 * scale, s, s, color);
}

static void lcd_draw_dot7(int x, int y, int scale, uint16_t color, uint16_t bg) {
    lcd_fill_rect(x, y, 6 * scale, 32 * scale, bg);
    lcd_fill_rect(x, y + 26 * scale, 4 * scale, 4 * scale, color);
}

static void lcd_clear(uint16_t color) {
    lcd_fill_rect(0, 0, LCD_W, LCD_H, color);
}

static void lcd_draw_pixel(uint16_t x, uint16_t y, uint16_t color) {
    if (x >= LCD_W || y >= LCD_H) return;
    lcd_set_addr_window(x, y, x, y);
    uint16_t swapped = (uint16_t)((color << 8) | (color >> 8));
    lcd_send_data(&swapped, 2);
}

/* Better approach: large readable numeric bars without needing a full font */
static void lcd_draw_bar(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                         float value, float minv, float maxv, uint16_t color) {
    lcd_fill_rect(x, y, w, h, COLOR_DARKGREY);
    if (value < minv) value = minv;
    if (value > maxv) value = maxv;
    uint16_t fill = (uint16_t)(((value - minv) / (maxv - minv)) * w);
    lcd_fill_rect(x, y, fill, h, color);
}

static void lcd_draw_digit7(int x, int y, int scale, int digit, uint16_t color, uint16_t bg) {
    static const uint8_t map[10] = {
        0b1111110, // 0
        0b0110000, // 1
        0b1101101, // 2
        0b1111001, // 3
        0b0110011, // 4
        0b1011011, // 5
        0b1011111, // 6
        0b1110000, // 7
        0b1111111, // 8
        0b1111011  // 9
    };

    int w = 18 * scale;
    int h = 32 * scale;
    int t = 4 * scale;
    int v = (h - 3 * t) / 2;

    lcd_fill_rect(x, y, w, h, bg);

    uint8_t m = map[digit];
    if (m & (1 << 6)) lcd_draw_hseg(x + t,     y,         w - 2 * t, t, color);           // a
    if (m & (1 << 5)) lcd_draw_vseg(x + w - t, y + t,     t, v, color);                   // b
    if (m & (1 << 4)) lcd_draw_vseg(x + w - t, y + 2*t+v, t, v, color);                   // c
    if (m & (1 << 3)) lcd_draw_hseg(x + t,     y + h - t, w - 2 * t, t, color);           // d
    if (m & (1 << 2)) lcd_draw_vseg(x,         y + 2*t+v, t, v, color);                   // e
    if (m & (1 << 1)) lcd_draw_vseg(x,         y + t,     t, v, color);                   // f
    if (m & (1 << 0)) lcd_draw_hseg(x + t,     y + t + v, w - 2 * t, t, color);           // g
}

static void lcd_draw_number_1dp(int x, int y, int scale, float value, uint16_t color, uint16_t bg) {
    if (value < 0) value = 0;
    int v10 = (int)(value * 10.0f + 0.5f);   // one decimal place
    int hundreds = (v10 / 1000) % 10;
    int tens     = (v10 / 100) % 10;
    int ones     = (v10 / 10) % 10;
    int tenths   = v10 % 10;

    int dx = 22 * scale;

    if (hundreds > 0) {
        lcd_draw_digit7(x, y, scale, hundreds, color, bg);
    }
    lcd_draw_digit7(x + dx,     y, scale, tens,   color, bg);
    lcd_draw_digit7(x + 2*dx,   y, scale, ones,   color, bg);
    lcd_draw_dot7  (x + 3*dx-6, y, scale, color, bg);
    lcd_draw_digit7(x + 3*dx,   y, scale, tenths, color, bg);
}

static void lcd_draw_number_int(int x, int y, int scale, int value, uint16_t color, uint16_t bg) {
    if (value < 0) value = 0;
    if (value > 9999) value = 9999;

    int d0 = (value / 1000) % 10;
    int d1 = (value / 100) % 10;
    int d2 = (value / 10) % 10;
    int d3 = value % 10;

    int dx = 22 * scale;
    int started = 0;
    lcd_fill_rect(x, y, 4 * dx, 32 * scale, bg);

    if (d0 || started) { lcd_draw_digit7(x + 0*dx, y, scale, d0, color, bg); started = 1; }
    if (d1 || started) { lcd_draw_digit7(x + 1*dx, y, scale, d1, color, bg); started = 1; }
    if (d2 || started) { lcd_draw_digit7(x + 2*dx, y, scale, d2, color, bg); started = 1; }
    lcd_draw_digit7(x + 3*dx, y, scale, d3, color, bg);
}

static bool lcd_get_glyph_5x7(char c, uint8_t rows[7]) {
    switch (c) {
    case 'A': { uint8_t r[7] = {0x0E, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11}; memcpy(rows, r, sizeof(r)); return true; }
    case 'B': { uint8_t r[7] = {0x1E, 0x11, 0x11, 0x1E, 0x11, 0x11, 0x1E}; memcpy(rows, r, sizeof(r)); return true; }
    case 'C': { uint8_t r[7] = {0x0E, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0E}; memcpy(rows, r, sizeof(r)); return true; }
    case 'D': { uint8_t r[7] = {0x1E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1E}; memcpy(rows, r, sizeof(r)); return true; }
    case 'E': { uint8_t r[7] = {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x1F}; memcpy(rows, r, sizeof(r)); return true; }
    case 'F': { uint8_t r[7] = {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x10}; memcpy(rows, r, sizeof(r)); return true; }
    case 'G': { uint8_t r[7] = {0x0E, 0x11, 0x10, 0x10, 0x13, 0x11, 0x0E}; memcpy(rows, r, sizeof(r)); return true; }
    case 'H': { uint8_t r[7] = {0x11, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11}; memcpy(rows, r, sizeof(r)); return true; }
    case 'I': { uint8_t r[7] = {0x0E, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0E}; memcpy(rows, r, sizeof(r)); return true; }
    case 'J': { uint8_t r[7] = {0x07, 0x02, 0x02, 0x02, 0x12, 0x12, 0x0C}; memcpy(rows, r, sizeof(r)); return true; }
    case 'K': { uint8_t r[7] = {0x11, 0x12, 0x14, 0x18, 0x14, 0x12, 0x11}; memcpy(rows, r, sizeof(r)); return true; }
    case 'L': { uint8_t r[7] = {0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1F}; memcpy(rows, r, sizeof(r)); return true; }
    case 'M': { uint8_t r[7] = {0x11, 0x1B, 0x15, 0x15, 0x11, 0x11, 0x11}; memcpy(rows, r, sizeof(r)); return true; }
    case 'N': { uint8_t r[7] = {0x11, 0x19, 0x15, 0x13, 0x11, 0x11, 0x11}; memcpy(rows, r, sizeof(r)); return true; }
    case 'O': { uint8_t r[7] = {0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E}; memcpy(rows, r, sizeof(r)); return true; }
    case 'P': { uint8_t r[7] = {0x1E, 0x11, 0x11, 0x1E, 0x10, 0x10, 0x10}; memcpy(rows, r, sizeof(r)); return true; }
    case 'Q': { uint8_t r[7] = {0x0E, 0x11, 0x11, 0x11, 0x15, 0x12, 0x0D}; memcpy(rows, r, sizeof(r)); return true; }
    case 'R': { uint8_t r[7] = {0x1E, 0x11, 0x11, 0x1E, 0x14, 0x12, 0x11}; memcpy(rows, r, sizeof(r)); return true; }
    case 'S': { uint8_t r[7] = {0x0F, 0x10, 0x10, 0x0E, 0x01, 0x01, 0x1E}; memcpy(rows, r, sizeof(r)); return true; }
    case 'T': { uint8_t r[7] = {0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04}; memcpy(rows, r, sizeof(r)); return true; }
    case 'U': { uint8_t r[7] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E}; memcpy(rows, r, sizeof(r)); return true; }
    case 'V': { uint8_t r[7] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x0A, 0x04}; memcpy(rows, r, sizeof(r)); return true; }
    case 'W': { uint8_t r[7] = {0x11, 0x11, 0x11, 0x15, 0x15, 0x15, 0x0A}; memcpy(rows, r, sizeof(r)); return true; }
    case 'X': { uint8_t r[7] = {0x11, 0x11, 0x0A, 0x04, 0x0A, 0x11, 0x11}; memcpy(rows, r, sizeof(r)); return true; }
    case 'Y': { uint8_t r[7] = {0x11, 0x11, 0x0A, 0x04, 0x04, 0x04, 0x04}; memcpy(rows, r, sizeof(r)); return true; }
    case 'Z': { uint8_t r[7] = {0x1F, 0x01, 0x02, 0x04, 0x08, 0x10, 0x1F}; memcpy(rows, r, sizeof(r)); return true; }

    case 'a': { uint8_t r[7] = {0x00, 0x00, 0x0E, 0x01, 0x0F, 0x11, 0x0F}; memcpy(rows, r, sizeof(r)); return true; }
    case 'b': { uint8_t r[7] = {0x10, 0x10, 0x1E, 0x11, 0x11, 0x11, 0x1E}; memcpy(rows, r, sizeof(r)); return true; }
    case 'c': { uint8_t r[7] = {0x00, 0x00, 0x0E, 0x10, 0x10, 0x11, 0x0E}; memcpy(rows, r, sizeof(r)); return true; }
    case 'd': { uint8_t r[7] = {0x01, 0x01, 0x0F, 0x11, 0x11, 0x11, 0x0F}; memcpy(rows, r, sizeof(r)); return true; }
    case 'e': { uint8_t r[7] = {0x00, 0x00, 0x0E, 0x11, 0x1F, 0x10, 0x0E}; memcpy(rows, r, sizeof(r)); return true; }
    case 'f': { uint8_t r[7] = {0x06, 0x08, 0x08, 0x1E, 0x08, 0x08, 0x08}; memcpy(rows, r, sizeof(r)); return true; }
    case 'g': { uint8_t r[7] = {0x00, 0x00, 0x0F, 0x11, 0x0F, 0x01, 0x0E}; memcpy(rows, r, sizeof(r)); return true; }
    case 'h': { uint8_t r[7] = {0x10, 0x10, 0x1E, 0x11, 0x11, 0x11, 0x11}; memcpy(rows, r, sizeof(r)); return true; }
    case 'i': { uint8_t r[7] = {0x04, 0x00, 0x0C, 0x04, 0x04, 0x04, 0x0E}; memcpy(rows, r, sizeof(r)); return true; }
    case 'j': { uint8_t r[7] = {0x02, 0x00, 0x06, 0x02, 0x02, 0x12, 0x0C}; memcpy(rows, r, sizeof(r)); return true; }
    case 'k': { uint8_t r[7] = {0x10, 0x10, 0x12, 0x14, 0x18, 0x14, 0x12}; memcpy(rows, r, sizeof(r)); return true; }
    case 'l': { uint8_t r[7] = {0x0C, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0E}; memcpy(rows, r, sizeof(r)); return true; }
    case 'm': { uint8_t r[7] = {0x00, 0x00, 0x1A, 0x15, 0x15, 0x15, 0x15}; memcpy(rows, r, sizeof(r)); return true; }
    case 'n': { uint8_t r[7] = {0x00, 0x00, 0x1E, 0x11, 0x11, 0x11, 0x11}; memcpy(rows, r, sizeof(r)); return true; }
    case 'o': { uint8_t r[7] = {0x00, 0x00, 0x0E, 0x11, 0x11, 0x11, 0x0E}; memcpy(rows, r, sizeof(r)); return true; }
    case 'p': { uint8_t r[7] = {0x00, 0x00, 0x1E, 0x11, 0x1E, 0x10, 0x10}; memcpy(rows, r, sizeof(r)); return true; }
    case 'q': { uint8_t r[7] = {0x00, 0x00, 0x0F, 0x11, 0x0F, 0x01, 0x01}; memcpy(rows, r, sizeof(r)); return true; }
    case 'r': { uint8_t r[7] = {0x00, 0x00, 0x16, 0x19, 0x10, 0x10, 0x10}; memcpy(rows, r, sizeof(r)); return true; }
    case 's': { uint8_t r[7] = {0x00, 0x00, 0x0F, 0x10, 0x0E, 0x01, 0x1E}; memcpy(rows, r, sizeof(r)); return true; }
    case 't': { uint8_t r[7] = {0x08, 0x08, 0x1E, 0x08, 0x08, 0x09, 0x06}; memcpy(rows, r, sizeof(r)); return true; }
    case 'u': { uint8_t r[7] = {0x00, 0x00, 0x11, 0x11, 0x11, 0x13, 0x0D}; memcpy(rows, r, sizeof(r)); return true; }
    case 'v': { uint8_t r[7] = {0x00, 0x00, 0x11, 0x11, 0x11, 0x0A, 0x04}; memcpy(rows, r, sizeof(r)); return true; }
    case 'w': { uint8_t r[7] = {0x00, 0x00, 0x11, 0x15, 0x15, 0x15, 0x0A}; memcpy(rows, r, sizeof(r)); return true; }
    case 'x': { uint8_t r[7] = {0x00, 0x00, 0x11, 0x0A, 0x04, 0x0A, 0x11}; memcpy(rows, r, sizeof(r)); return true; }
    case 'y': { uint8_t r[7] = {0x00, 0x00, 0x11, 0x11, 0x0F, 0x01, 0x0E}; memcpy(rows, r, sizeof(r)); return true; }
    case 'z': { uint8_t r[7] = {0x00, 0x00, 0x1F, 0x02, 0x04, 0x08, 0x1F}; memcpy(rows, r, sizeof(r)); return true; }

    case '0': { uint8_t r[7] = {0x0E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0E}; memcpy(rows, r, sizeof(r)); return true; }
    case '1': { uint8_t r[7] = {0x04, 0x0C, 0x14, 0x04, 0x04, 0x04, 0x1F}; memcpy(rows, r, sizeof(r)); return true; }
    case '2': { uint8_t r[7] = {0x0E, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1F}; memcpy(rows, r, sizeof(r)); return true; }
    case '3': { uint8_t r[7] = {0x0E, 0x11, 0x01, 0x06, 0x01, 0x11, 0x0E}; memcpy(rows, r, sizeof(r)); return true; }
    case '4': { uint8_t r[7] = {0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02}; memcpy(rows, r, sizeof(r)); return true; }
    case '5': { uint8_t r[7] = {0x1F, 0x10, 0x1E, 0x01, 0x01, 0x11, 0x0E}; memcpy(rows, r, sizeof(r)); return true; }
    case '6': { uint8_t r[7] = {0x06, 0x08, 0x10, 0x1E, 0x11, 0x11, 0x0E}; memcpy(rows, r, sizeof(r)); return true; }
    case '7': { uint8_t r[7] = {0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08}; memcpy(rows, r, sizeof(r)); return true; }
    case '8': { uint8_t r[7] = {0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E}; memcpy(rows, r, sizeof(r)); return true; }
    case '9': { uint8_t r[7] = {0x0E, 0x11, 0x11, 0x0F, 0x01, 0x02, 0x0C}; memcpy(rows, r, sizeof(r)); return true; }

    case '!': { uint8_t r[7] = {0x04, 0x04, 0x04, 0x04, 0x04, 0x00, 0x04}; memcpy(rows, r, sizeof(r)); return true; }
    case '"': { uint8_t r[7] = {0x0A, 0x0A, 0x0A, 0x00, 0x00, 0x00, 0x00}; memcpy(rows, r, sizeof(r)); return true; }
    case '#': { uint8_t r[7] = {0x0A, 0x0A, 0x1F, 0x0A, 0x1F, 0x0A, 0x0A}; memcpy(rows, r, sizeof(r)); return true; }
    case '$': { uint8_t r[7] = {0x04, 0x0F, 0x14, 0x0E, 0x05, 0x1E, 0x04}; memcpy(rows, r, sizeof(r)); return true; }
    case '%': { uint8_t r[7] = {0x19, 0x19, 0x02, 0x04, 0x08, 0x13, 0x13}; memcpy(rows, r, sizeof(r)); return true; }
    case '&': { uint8_t r[7] = {0x0C, 0x12, 0x14, 0x08, 0x15, 0x12, 0x0D}; memcpy(rows, r, sizeof(r)); return true; }
    case '\'': { uint8_t r[7] = {0x04, 0x04, 0x08, 0x00, 0x00, 0x00, 0x00}; memcpy(rows, r, sizeof(r)); return true; }
    case '(': { uint8_t r[7] = {0x02, 0x04, 0x08, 0x08, 0x08, 0x04, 0x02}; memcpy(rows, r, sizeof(r)); return true; }
    case ')': { uint8_t r[7] = {0x08, 0x04, 0x02, 0x02, 0x02, 0x04, 0x08}; memcpy(rows, r, sizeof(r)); return true; }
    case '*': { uint8_t r[7] = {0x00, 0x04, 0x15, 0x0E, 0x15, 0x04, 0x00}; memcpy(rows, r, sizeof(r)); return true; }
    case '+': { uint8_t r[7] = {0x00, 0x04, 0x04, 0x1F, 0x04, 0x04, 0x00}; memcpy(rows, r, sizeof(r)); return true; }
    case ',': { uint8_t r[7] = {0x00, 0x00, 0x00, 0x00, 0x04, 0x04, 0x08}; memcpy(rows, r, sizeof(r)); return true; }
    case '-': { uint8_t r[7] = {0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00}; memcpy(rows, r, sizeof(r)); return true; }
    case '.': { uint8_t r[7] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04}; memcpy(rows, r, sizeof(r)); return true; }
    case '/': { uint8_t r[7] = {0x01, 0x01, 0x02, 0x04, 0x08, 0x10, 0x10}; memcpy(rows, r, sizeof(r)); return true; }
    case ':': { uint8_t r[7] = {0x00, 0x04, 0x00, 0x00, 0x04, 0x00, 0x00}; memcpy(rows, r, sizeof(r)); return true; }
    case ';': { uint8_t r[7] = {0x00, 0x04, 0x00, 0x00, 0x04, 0x04, 0x08}; memcpy(rows, r, sizeof(r)); return true; }
    case '<': { uint8_t r[7] = {0x01, 0x02, 0x04, 0x08, 0x04, 0x02, 0x01}; memcpy(rows, r, sizeof(r)); return true; }
    case '=': { uint8_t r[7] = {0x00, 0x00, 0x1F, 0x00, 0x1F, 0x00, 0x00}; memcpy(rows, r, sizeof(r)); return true; }
    case '>': { uint8_t r[7] = {0x10, 0x08, 0x04, 0x02, 0x04, 0x08, 0x10}; memcpy(rows, r, sizeof(r)); return true; }
    case '?': { uint8_t r[7] = {0x0E, 0x11, 0x01, 0x02, 0x04, 0x00, 0x04}; memcpy(rows, r, sizeof(r)); return true; }
    case '@': { uint8_t r[7] = {0x0E, 0x11, 0x15, 0x1D, 0x10, 0x11, 0x0E}; memcpy(rows, r, sizeof(r)); return true; }
    case '[': { uint8_t r[7] = {0x0E, 0x08, 0x08, 0x08, 0x08, 0x08, 0x0E}; memcpy(rows, r, sizeof(r)); return true; }
    case '\\': { uint8_t r[7] = {0x10, 0x10, 0x08, 0x04, 0x02, 0x01, 0x01}; memcpy(rows, r, sizeof(r)); return true; }
    case ']': { uint8_t r[7] = {0x0E, 0x02, 0x02, 0x02, 0x02, 0x02, 0x0E}; memcpy(rows, r, sizeof(r)); return true; }
    case '^': { uint8_t r[7] = {0x04, 0x0A, 0x11, 0x00, 0x00, 0x00, 0x00}; memcpy(rows, r, sizeof(r)); return true; }
    case '_': { uint8_t r[7] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F}; memcpy(rows, r, sizeof(r)); return true; }
    case '`': { uint8_t r[7] = {0x08, 0x04, 0x02, 0x00, 0x00, 0x00, 0x00}; memcpy(rows, r, sizeof(r)); return true; }
    case '{': { uint8_t r[7] = {0x02, 0x04, 0x04, 0x08, 0x04, 0x04, 0x02}; memcpy(rows, r, sizeof(r)); return true; }
    case '|': { uint8_t r[7] = {0x04, 0x04, 0x04, 0x00, 0x04, 0x04, 0x04}; memcpy(rows, r, sizeof(r)); return true; }
    case '}': { uint8_t r[7] = {0x08, 0x04, 0x04, 0x02, 0x04, 0x04, 0x08}; memcpy(rows, r, sizeof(r)); return true; }
    case '~': { uint8_t r[7] = {0x00, 0x00, 0x09, 0x16, 0x00, 0x00, 0x00}; memcpy(rows, r, sizeof(r)); return true; }
    case ' ': { uint8_t r[7] = {0, 0, 0, 0, 0, 0, 0}; memcpy(rows, r, sizeof(r)); return true; }
    default:
        return false; 
    }
}

static void lcd_draw_char_block(uint16_t x, uint16_t y, char c, uint16_t fg, uint16_t bg) {
    uint8_t rows[7] = {0};
    bool has_glyph = lcd_get_glyph_5x7(c, rows);

    lcd_fill_rect(x, y, 6, 8, bg);
    if (!has_glyph) {
        return;
    }

    for (int ry = 0; ry < 7; ry++) {
        for (int rx = 0; rx < 5; rx++) {
            if (rows[ry] & (1u << (4 - rx))) {
                lcd_draw_pixel((uint16_t)(x + rx), (uint16_t)(y + ry), fg);
            }
        }
    }
}

static void lcd_draw_char_block_scaled(uint16_t x, uint16_t y, char c, uint8_t scale,
                                       uint16_t fg, uint16_t bg) {
    if (scale <= 1) {
        lcd_draw_char_block(x, y, c, fg, bg);
        return;
    }

    uint8_t rows[7] = {0};
    bool has_glyph = lcd_get_glyph_5x7(c, rows);
    uint16_t cw = (uint16_t)(6 * scale);
    uint16_t ch = (uint16_t)(8 * scale);
    lcd_fill_rect(x, y, cw, ch, bg);
    if (!has_glyph) {
        return;
    }

    for (int ry = 0; ry < 7; ry++) {
        for (int rx = 0; rx < 5; rx++) {
            if (rows[ry] & (1u << (4 - rx))) {
                lcd_fill_rect((uint16_t)(x + rx * scale), (uint16_t)(y + ry * scale),
                              scale, scale, fg);
            }
        }
    }
}

static void lcd_draw_string_block(uint16_t x, uint16_t y, const char *s, uint16_t fg, uint16_t bg) {
    while (*s) {
        if (*s == ' ') {
            lcd_fill_rect(x, y, 6, 8, bg);
        } else {
            lcd_draw_char_block(x, y, *s, fg, bg);
        }
        x += 6;
        s++;
    }
}

static void lcd_draw_string_block_scaled(uint16_t x, uint16_t y, const char *s, uint8_t scale,
                                         uint16_t fg, uint16_t bg) {
    while (*s) {
        lcd_draw_char_block_scaled(x, y, *s, scale, fg, bg);
        x = (uint16_t)(x + 6 * scale);
        s++;
    }
}


static void lcd_write_label_value(uint16_t y, const char *label, float value, const char *unit,
                                  uint16_t color) {
    char buf[48];
    snprintf(buf, sizeof(buf), "%s %.1f %s", label, value, unit);
    lcd_draw_string_block(10, y, buf, color, COLOR_BLACK);
}

static void lcd_write_status(uint16_t y, const char *msg, uint16_t color) {
    lcd_draw_string_block(10, y, msg, color, COLOR_BLACK);
}

static void lcd_render_packet(const sensor_packet_t *pkt) {
    static bool first_draw = true;
    static int prev_meat = -100000;
    static int prev_pan_tenths = -100000;
    static int prev_overtemp = -1;
    static int prev_blink = -1;

    static int prev_probe_disconnected = -1;
    static int prev_tc_disconnected = -1;

    int meat = (int)(pkt->food_probe_c + 0.5f);
    int pan_tenths = (int)(pkt->thermocouple_c * 10.0f + (pkt->thermocouple_c >= 0 ? 0.5f : -0.5f));
    int overtemp = pkt->thermocouple_c > PAN_MAX_TEMP_C ? 1 : 0;
    int blink_on = ((esp_timer_get_time() / 350000) % 2) == 0 ? 1 : 0;

    int probe_disconnected = (pkt->control_flags & CTRL_FLAG_PROBE_DISCONNECT) ? 1 : 0;
    int tc_disconnected = ((pkt->control_flags & CTRL_FLAG_TC_DISCONNECT) || (pkt->thermocouple_c <= 0.0f)) ? 1 : 0;

    if (first_draw) {
        lcd_clear(COLOR_BLACK);
        lcd_fill_rect(0, 0, LCD_W, 155, 0x0841);
        lcd_fill_rect(0, 160, LCD_W, 40, 0x10A2);
        lcd_fill_rect(0, 205, LCD_W, 35, COLOR_BLACK);
        lcd_write_status(10, "MEAT PROBE C", COLOR_WHITE);
        lcd_write_status(170, "PAN TEMP C", COLOR_WHITE);
        first_draw = false;
    }

    if (probe_disconnected) {
        if (prev_probe_disconnected != probe_disconnected || prev_blink != blink_on) {
            uint16_t bg = blink_on ? 0x0841 : COLOR_BLACK;
            uint16_t fg = COLOR_RED;
            lcd_fill_rect(0, 0, LCD_W, 155, bg);
            lcd_write_status(10, "MEAT PROBE C", COLOR_WHITE);
            // lcd_draw_string_block(65, 75, "FOOD PROBE DISCONNECT", fg, bg);
            lcd_draw_string_block_scaled(70, 70, "DISCONNECT", 2, fg, bg);
        }
    } else if (prev_probe_disconnected != probe_disconnected) {
        lcd_fill_rect(0, 0, LCD_W, 155, 0x0841);
        lcd_write_status(10, "MEAT PROBE C", COLOR_WHITE);
        prev_meat = -100000;
    }

    if (!probe_disconnected && meat != prev_meat) {
        lcd_draw_number_int(22, 42, 3, meat, COLOR_GREEN, 0x0841);
        prev_meat = meat;
    }

    if (tc_disconnected) {
        if (prev_tc_disconnected != tc_disconnected || prev_blink != blink_on) {
            uint16_t bg = blink_on ? 0x10A2 : COLOR_BLACK;
            uint16_t fg = COLOR_RED;
            lcd_fill_rect(0, 160, LCD_W, 40, bg);
            lcd_write_status(170, "PAN TEMP C", COLOR_WHITE);
            // lcd_draw_string_block(100, 180, "PAN TEMPERATURE DISCONNECT", fg, bg);
            lcd_draw_string_block_scaled(100, 170, "DISCONNECT", 3, fg, bg);
        }
    } else if (prev_tc_disconnected != tc_disconnected) {
        lcd_fill_rect(0, 160, LCD_W, 40, 0x10A2);
        lcd_write_status(170, "PAN TEMP C", COLOR_WHITE);
        prev_pan_tenths = -100000;
    }

    if (!tc_disconnected && pan_tenths != prev_pan_tenths) {
        lcd_draw_number_1dp(170, 166, 1, pkt->thermocouple_c, COLOR_YELLOW, 0x10A2);
        prev_pan_tenths = pan_tenths;
    }

    if (overtemp) {
        if (prev_overtemp != overtemp || prev_blink != blink_on) {
            uint16_t bg = blink_on ? COLOR_RED : COLOR_YELLOW;
            uint16_t fg = blink_on ? COLOR_WHITE : COLOR_BLACK;
            lcd_fill_rect(0, 205, LCD_W, 35, bg);
            lcd_draw_string_block(130, 218, "OVERTEMP", fg, bg);
        }
    } else if (prev_overtemp != overtemp) {
        lcd_fill_rect(0, 205, LCD_W, 35, COLOR_BLACK);
    }

    prev_overtemp = overtemp;
    prev_blink = blink_on;

    prev_probe_disconnected = probe_disconnected;
    prev_tc_disconnected = tc_disconnected;
}

static void lcd_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_LCD_DC) | (1ULL << PIN_LCD_RST) | (1ULL << PIN_LCD_BL),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    gpio_set_level(PIN_LCD_BL, 1);
    gpio_set_level(PIN_LCD_DC, 1);
    gpio_set_level(PIN_LCD_RST, 1);

    lcd_reset_hw();

    lcd_send_cmd(0x01); /* software reset */
    vTaskDelay(pdMS_TO_TICKS(120));

    lcd_send_cmd(0x11); /* sleep out */
    vTaskDelay(pdMS_TO_TICKS(120));

    lcd_send_cmd(0x3A); /* pixel format */
    lcd_send_u8(0x55);  /* RGB565 */

    lcd_send_cmd(0x36);
    lcd_send_u8(0x28);

    lcd_send_cmd(0x29); /* display on */
    vTaskDelay(pdMS_TO_TICKS(20));

    lcd_clear(COLOR_BLACK);
}

/* --------------------------- SPI setup --------------------------- */

static esp_err_t spi_bus_init_once(void) {
    if (g_spi_bus_ready) {
        return ESP_OK;
    }

    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = PIN_MISO,
        .sclk_io_num = PIN_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_W * 40 * 2
    };

    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
    g_spi_bus_ready = true;
    return ESP_OK;
}

static esp_err_t max31855_init(void) {
    ESP_ERROR_CHECK(spi_bus_init_once());

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1000000,
        .mode = 0,
        .spics_io_num = PIN_MAX_CS,
        .queue_size = 1,
        .flags = SPI_DEVICE_HALFDUPLEX
    };
    return spi_bus_add_device(SPI2_HOST, &devcfg, &g_maxdev);
}

static esp_err_t lcd_spi_add_device(void) {
    ESP_ERROR_CHECK(spi_bus_init_once());

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 10000000,
        .mode = 0,
        .spics_io_num = PIN_LCD_CS,
        .queue_size = 4,
        .flags = SPI_DEVICE_NO_DUMMY
    };
    return spi_bus_add_device(SPI2_HOST, &devcfg, &g_lcddev);
}

/* --------------------------- MAX31855 --------------------------- */

static esp_err_t max31855_read_raw(uint32_t *raw_out) {
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

static float max31855_tctemp_fault(uint32_t raw, bool *fault_out) {
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

static float max31855_internal(uint32_t raw) {
    int32_t intern_temp = (int32_t)((raw >> 4) & 0x0FFFu);
    if (intern_temp & 0x0800) {
        intern_temp |= ~0x0FFF;
    }
    return (float)intern_temp * 0.0625f;
}

static sensor_packet_t collect_sensor_packet(void) {
    sensor_packet_t pkt;
    taskENTER_CRITICAL(&g_pkt_lock);
    pkt = g_latest_pkt;
    taskEXIT_CRITICAL(&g_pkt_lock);
    pkt.seq = g_seq++;
    return pkt;
}

/* --------------------------- tasks --------------------------- */

static float thermistor_conv(float raw) {
    const float ADC_MAX = 4095.0f;
    const float RFIXED = 100000.0f;
    const float VCC = 3.295f;
    const float A = -0.1943e-3f;
    const float B = 3.4023e-4f;
    const float C = -2.3843e-7f;

    int adc_raw = (int)raw;
    if (adc_raw < 1) adc_raw = 1;
    if (adc_raw > (int)(ADC_MAX - 1.0f)) adc_raw = (int)(ADC_MAX - 1.0f);

    float vout = ((float)adc_raw) * VCC / ADC_MAX;
    float rth = vout * RFIXED / (VCC - vout);
    float ln_rth = logf(rth);
    float tinv = A + B * ln_rth + C * ln_rth * ln_rth * ln_rth;
    float temp_k = 1.0f / tinv;
    return temp_k - 273.15f;
}


static void sensor_task(void *param) {
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
        uint8_t tc_fault_flags = (uint8_t)(raw_tc & 0x07u);

        for (uint32_t i = 0; i < num_parsed_samples; i++) {
            if (!parsed_data[i].valid) continue;
            if ((++decim % DECIM_N) != 0) continue;

            bool probe_disconnected = (parsed_data[i].raw_data >= 4095u);
            bool tc_disconnected = ((tc_fault_flags & 0x01u) != 0) || (tc <= 0.0f);

            sensor_packet_t next = {
                .seq = 0,
                .timestamp_us = (uint64_t)esp_timer_get_time(),
                .thermocouple_c = tc,
                .food_probe_c = thermistor_conv((float)parsed_data[i].raw_data),
                .control_flags = (uint8_t)((tc_fault ? CTRL_FLAG_TC_FAULT : 0u) |
                            CTRL_FLAG_SAMPLE_VALID |
                            (probe_disconnected ? CTRL_FLAG_PROBE_DISCONNECT : 0u) |
                            (tc_disconnected ? CTRL_FLAG_TC_DISCONNECT : 0u)),
                .tc_fault_flags = tc_fault_flags,
                .reserved = {0, 0},
            };

            taskENTER_CRITICAL(&g_pkt_lock);
            g_latest_pkt = next;
            taskEXIT_CRITICAL(&g_pkt_lock);

            stepper_handle_overtemp(next.thermocouple_c);

            ESP_LOGI(TAG,
                     "%" PRIu64 ",meat=%.2fC raw_adc=%" PRIu32 " raw_tc=0x%08" PRIX32 " pan=%.2fC intern=%.2fC fault=%d flags=0x%02X",
                     next.timestamp_us, next.food_probe_c, parsed_data[i].raw_data, raw_tc, tc, tc_internal, tc_fault, tc_fault_flags);
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

static void lcd_task(void *param) {
    (void)param;

    ESP_ERROR_CHECK(lcd_spi_add_device());
    lcd_init();

    while (1) {
        sensor_packet_t pkt;
        taskENTER_CRITICAL(&g_pkt_lock);
        pkt = g_latest_pkt;
        taskEXIT_CRITICAL(&g_pkt_lock);

        lcd_render_packet(&pkt);
        vTaskDelay(pdMS_TO_TICKS(120));
    }
}

/* --------------------------- BLE --------------------------- */

static int gatt_svr_chr_access_cb(uint16_t conn_handle,
                                  uint16_t attr_handle,
                                  struct ble_gatt_access_ctxt *ctxt,
                                  void *arg) {
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

static int ble_gap_event(struct ble_gap_event *event, void *arg) {
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

static void ble_app_advertise(void) {
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

    rc = ble_gap_adv_start(g_own_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_start failed: %d", rc);
    }
}

static void ble_on_sync(void) {
    int rc = ble_hs_id_infer_auto(0, &g_own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_hs_id_infer_auto failed: %d", rc);
        return;
    }
    ble_app_advertise();
}

static void host_task(void *param) {
    (void)param;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

static void notify_task(void *param) {
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

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(spi_bus_init_once());

    nimble_port_init();
    ble_svc_gap_init();
    ble_svc_gatt_init();

    int rc = ble_gatts_count_cfg(gatt_svcs);
    ESP_ERROR_CHECK(rc == 0 ? ESP_OK : ESP_FAIL);
    rc = ble_gatts_add_svcs(gatt_svcs);
    ESP_ERROR_CHECK(rc == 0 ? ESP_OK : ESP_FAIL);

    ble_svc_gap_device_name_set(DEVICE_NAME);
    ble_hs_cfg.sync_cb = ble_on_sync;

    xTaskCreate(sensor_task, "sensor_task", 8192, NULL, 6, NULL);
    xTaskCreate(lcd_task, "lcd_task", 6144, NULL, 4, NULL);
    nimble_port_freertos_init(host_task);
    xTaskCreate(notify_task, "notify_task", 4096, NULL, 5, NULL);
}