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
#define PIN_CS                  10
#define ADC_INPUT_GPIO          GPIO_NUM_1

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

static esp_err_t max31855_init(void)
{
    spi_bus_config_t buscfg = {
        .mosi_io_num = -1,
        .miso_io_num = PIN_MISO,
        .sclk_io_num = PIN_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4
    };

    spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1000000,
        .mode = 0,
        .spics_io_num = PIN_CS,
        .queue_size = 1,
        .flags = SPI_DEVICE_HALFDUPLEX
    };
    return spi_bus_add_device(SPI2_HOST, &devcfg, &g_maxdev);
}

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

    xTaskCreate(sensor_task, "sensor_task", 8192, NULL, 6, NULL);
    nimble_port_freertos_init(host_task);
    xTaskCreate(notify_task, "notify_task", 4096, NULL, 5, NULL);
}
