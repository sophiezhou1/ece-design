// adc
#include <stdio.h>
#include <inttypes.h>
#include "driver/spi_common.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_continuous.h"
#include "esp_err.h"
#include "hal/adc_types.h"
#include "hal/spi_types.h"
#include "lwip/err.h"
#include "soc/gpio_num.h"
#include "esp_timer.h"


// spi
#include <stdint.h>
#include <string.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define PIN_SCK   12
#define PIN_MISO  13
#define PIN_CS    10

esp_err_t err;



/*
!!! CENTER RAIL IS 5V !!!
esp32 [4] -> uln in [1] -> uln out [18] -> stepper (blue)
esp32 [5] -> uln in [2] -> uln out [17] -> stepper (pink)
esp32 [6] -> uln in [3] -> uln out [16] -> stepper (yellow)
esp32 [7] -> uln in [4] -> uln out [15] -> stepper (orange)
uln [10] -> stepper 5V (red)
uln [9] -> GND
*/

// static const int pins[4] = {4, 5, 6, 7};

static const int steps[4][4] = {
    {1,0,0,0},
    {0,1,0,0},
    {0,0,1,0},
    {0,0,0,1}
};

int step_counter = 0;

void app_main(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << 15) | (1ULL << 16) | (1ULL << 17) | (1ULL << 18),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    step_counter = 0;

    // 1800 steps for startup + 1 rotation
    while (step_counter < 1800) {
        for (int s = 0; s < 4; s++) {
            gpio_set_level(15, steps[s][0]);
            step_counter++;
            gpio_set_level(16, steps[s][1]);
            step_counter++;
            gpio_set_level(17, steps[s][2]);
            step_counter++;
            gpio_set_level(18, steps[s][3]);
            step_counter++;
            vTaskDelay(pdMS_TO_TICKS(20));
            // printf("%d      ", step_counter);
        }
    }

    printf("break");
    step_counter = 0;

    // 2050 for regular full rotation
    while (step_counter < 5) {
        for (int s = 0; s < 4; s++) {
            gpio_set_level(15, steps[s][0]);
            step_counter++;
            gpio_set_level(16, steps[s][1]);
            step_counter++;
            gpio_set_level(17, steps[s][2]);
            step_counter++;
            gpio_set_level(18, steps[s][3]);
            step_counter++;
            vTaskDelay(pdMS_TO_TICKS(20));
            printf("%d      ", step_counter);
        }
    }
}

