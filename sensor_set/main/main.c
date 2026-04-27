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

static spi_device_handle_t maxdev = NULL;
esp_err_t err;

static esp_err_t max31855_init(void) {
    spi_bus_config_t buscfg = {
        .mosi_io_num = -1, // not used
        .miso_io_num = PIN_MISO,
        .sclk_io_num = PIN_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4
    };

    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1000000,
        .mode = 0,
        .spics_io_num = PIN_CS,
        .queue_size = 1,
        .flags = SPI_DEVICE_HALFDUPLEX
    };
    err = spi_bus_add_device(SPI2_HOST, &devcfg, &maxdev);
    return err;
}

static esp_err_t max31855_read_raw(uint32_t *raw_out) {
    uint8_t rx[4] = {0};

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));

    t.length = 0;
    t.rxlength = 32;
    t.tx_buffer = NULL;
    t.rx_buffer = rx;

    esp_err_t err = spi_device_transmit(maxdev, &t);
    if (err != ERR_OK) return err;

    // msb first
    uint32_t raw = ((uint32_t)rx[0] << 24) |
                   ((uint32_t)rx[1] << 16) |
                   ((uint32_t)rx[2] <<  8) |
                   ((uint32_t)rx[3] <<  0);

    *raw_out = raw;
    return ERR_OK;
}

static float max31855_tctemp_fault(uint32_t raw, bool *fault_out) {
    // fault bit 16 (true if scv | scg | oc)
    bool fault = (raw >> 16) & 1u;
    if (fault_out) *fault_out = fault;

    // thermocouple temp bits [31:18]
    // 6400 digital out = 1600 C (0.25 C per LSB)
    int32_t t14 = (int32_t)((raw >> 18) & 0x3FFFu); // 18 bit shift, mask to 14 bits
    if (t14 & 0x2000) t14 |= ~0x3FFF;   // sign-extend
    return (float)t14 * 0.25f;
}

static float max31855_internal(uint32_t raw) {
    // internal temp bits [15:4]
    // 2032 digital out = 127 C (0.0625 C per LSB)
    int32_t intern_temp = (int32_t)((raw >> 4) & 0x0FFFu); // 4 bit shift, mask to 12 bits
    if (intern_temp & 0x0800) intern_temp |= ~0x0FFF;
    return (float)intern_temp * 0.0625f;
}


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


// void app_main(void)
// {
//     // adc init
//     setvbuf(stdout, NULL, _IONBF, 0);  // real-time prints

//     adc_unit_t unit;
//     adc_channel_t channel;
//     ESP_ERROR_CHECK(adc_continuous_io_to_channel(GPIO_NUM_1, &unit, &channel));

//     adc_continuous_handle_t handle = NULL;
//     adc_continuous_handle_cfg_t adc_config = {
//         .max_store_buf_size = 4096,
//         .conv_frame_size = 1024,
//     };
//     ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));

//     adc_digi_pattern_config_t pattern = {
//         .atten = ADC_ATTEN_DB_12,
//         .bit_width = ADC_BITWIDTH_12,
//         .channel = channel,
//         .unit = unit,
//     };

//     adc_continuous_config_t cfg = {
//         .sample_freq_hz = 2000,
//         .conv_mode = ADC_CONV_SINGLE_UNIT_1,
//         .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
//         .pattern_num = 1,
//         .adc_pattern = &pattern,
//     };

//     ESP_ERROR_CHECK(adc_continuous_config(handle, &cfg));
//     ESP_ERROR_CHECK(adc_continuous_start(handle));

//     uint8_t result[1024];

//     // allocate parsed_data once, not based on ret_num
//     static adc_continuous_data_t parsed_data[1024 / SOC_ADC_DIGI_RESULT_BYTES];

//     // Print at ~10 Hz by decimating samples
//     const uint32_t DECIM_N = 20;      // for 2000 Hz, DECIM_N = 200 -> 10 Hz
//     static uint32_t decim = 0;

    
    
//     // spi init
//     ESP_ERROR_CHECK(max31855_init());
    
    
    
//     while (true) {
//         // adc run
//         uint32_t ret_num = 0;
//         esp_err_t ret = adc_continuous_read(handle, result, sizeof(result), &ret_num, 1000);
//         if (ret != ESP_OK || ret_num == 0) {
//             vTaskDelay(pdMS_TO_TICKS(1));
//             continue;
//         }

//         uint32_t num_parsed_samples = 0;
//         esp_err_t parse_ret = adc_continuous_parse_data(
//             handle,
//             result,
//             ret_num,
//             parsed_data,
//             &num_parsed_samples
//         );

//         if (parse_ret != ESP_OK) {
//             vTaskDelay(pdMS_TO_TICKS(1));
//             continue;
//         }

//         // spi run
//         uint32_t raw_tc = 0;
//         ESP_ERROR_CHECK(max31855_read_raw(&raw_tc));

//         bool fault = false;
//         float tc = max31855_tctemp_fault(raw_tc, &fault);
//         float intern = max31855_internal(raw_tc);

//         // oc fault: 0, scg fault: 1, scv falt: 2
//         uint8_t fault_oc = (raw_tc >> 0) & 1u;
//         uint8_t fault_scg = (raw_tc >> 1) & 1u;
//         uint8_t fault_scv = (raw_tc >> 2) & 1u;

//         for (uint32_t i = 0; i < num_parsed_samples; i++) {
//             if (!parsed_data[i].valid) continue;

//             if ((++decim % DECIM_N) != 0) continue;

//             uint32_t raw = parsed_data[i].raw_data;
//             int64_t t_us = esp_timer_get_time();
//             // printf("%" PRId64 ",%" PRIu32 "\n", t_us, raw);
//             // printf("raw=0x%08" PRIX32 "  tc=%.2f C  intern=%.2f C  fault=%d (OC=%d SCG=%d SCV=%d)\n",
//             //     raw_tc, tc, intern, fault, fault_oc, fault_scg, fault_scv);
//             printf("%" PRId64 ",adc%" PRIu32 " raw=0x%08" PRIX32 "  tc=%.2f C  intern=%.2f C  fault=%d (OC=%d SCG=%d SCV=%d)\n", t_us, raw, raw_tc, tc, intern, fault, fault_oc, fault_scg, fault_scv);
//         }

//         vTaskDelay(pdMS_TO_TICKS(1));

//         // vTaskDelay(pdMS_TO_TICKS(250));
//     }
// }