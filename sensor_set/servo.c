#include <stdio.h>
#include <inttypes.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"

static const int out_pins[4] = {4, 5, 6, 7};
static const int in_pins[4]  = {17, 18, 19, 20};

static const int steps[4][4] = {
    {1, 0, 0, 0},
    {0, 1, 0, 0},
    {0, 0, 1, 0},
    {0, 0, 0, 1}
};

static bool pins_match(const int expected[4], int actual[4]) {
    for (int i = 0; i < 4; i++) {
        if (expected[i] != actual[i]) return false;
    }
    return true;
}

void app_main(void)
{
    // Configure output pins 4,5,6,7
    gpio_config_t out_conf = {
        .pin_bit_mask =
            (1ULL << out_pins[0]) |
            (1ULL << out_pins[1]) |
            (1ULL << out_pins[2]) |
            (1ULL << out_pins[3]),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&out_conf));

    // Configure input pins 17,18,19,20
    gpio_config_t in_conf = {
        .pin_bit_mask =
            (1ULL << in_pins[0]) |
            (1ULL << in_pins[1]) |
            (1ULL << in_pins[2]) |
            (1ULL << in_pins[3]),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&in_conf));

    // Initialize outputs low
    for (int i = 0; i < 4; i++) {
        gpio_set_level(out_pins[i], 0);
    }

    printf("Loopback timing test started\n");
    printf("Outputs: 4 5 6 7\n");
    printf("Inputs : 17 18 19 20\n");

    while (1) {
        for (int s = 0; s < 4; s++) {
            int actual[4] = {0, 0, 0, 0};

            // Apply output pattern
            for (int i = 0; i < 4; i++) {
                gpio_set_level(out_pins[i], steps[s][i]);
            }

            // Mark send time immediately after output write
            int64_t t_send_us = esp_timer_get_time();

            // Poll until inputs match outputs
            bool matched = false;
            int64_t t_match_us = 0;
            const int64_t timeout_us = 50000; // 50 ms timeout

            while ((esp_timer_get_time() - t_send_us) < timeout_us) {
                for (int i = 0; i < 4; i++) {
                    actual[i] = gpio_get_level(in_pins[i]);
                }

                if (pins_match(steps[s], actual)) {
                    t_match_us = esp_timer_get_time();
                    matched = true;
                    break;
                }

                esp_rom_delay_us(1);
            }

            if (matched) {
                printf("step %d | sent=%" PRId64 " us | in=[%d %d %d %d] | delay=%" PRId64 " us\n",
                       s,
                       t_send_us,
                       actual[0], actual[1], actual[2], actual[3],
                       t_match_us - t_send_us);
            } else {
                printf("step %d | TIMEOUT | expected=[%d %d %d %d] | last_in=[%d %d %d %d]\n",
                       s,
                       steps[s][0], steps[s][1], steps[s][2], steps[s][3],
                       actual[0], actual[1], actual[2], actual[3]);
            }

            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }
}