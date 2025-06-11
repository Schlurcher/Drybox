#include "sensor_read.h"
#include "event_bits.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "aht20.h"

// External references
extern EventGroupHandle_t sensor_bits;
extern i2c_master_bus_handle_t i2c_bus_0;
extern i2c_master_bus_handle_t i2c_bus_1;

static const char* TAG = "SENSOR_READ";

#define I2C_MASTER_FREQ_HZ 100000


QueueHandle_t queue_handle;
const int queue_element_size = 10;

typedef struct
{
    float temp0;
    float temp1;
    float hum0;
    float hum1;
} message_t;


void sensor_read_task(void* params)
{
    ESP_LOGI(TAG, "Started on Core: %d, Stack: %d", xPortGetCoreID(), uxTaskGetStackHighWaterMark(NULL));

    float temp0 = 0, hum0 = 0, temp1 = 0, hum1 = 0;

    // Define configuration once
    const i2c_aht20_config_t aht20_i2c_config = {
        .i2c_config.device_address = AHT20_ADDRESS_0,
        .i2c_config.scl_speed_hz = I2C_MASTER_FREQ_HZ,
        .i2c_timeout = 100,
    };

    // queue_handle = xQueueCreate(queue_element_size, sizeof(message_t));

    aht20_dev_handle_t aht20_handle_0 = NULL;
    aht20_dev_handle_t aht20_handle_1 = NULL;

    // Use the same config for both sensors
    ESP_ERROR_CHECK(aht20_new_sensor(i2c_bus_0, &aht20_i2c_config, &aht20_handle_0));
    ESP_ERROR_CHECK(aht20_new_sensor(i2c_bus_1, &aht20_i2c_config, &aht20_handle_1));

    while (1)
    {
        ESP_LOGI(TAG, "Core: %d, Stack: %d", xPortGetCoreID(), uxTaskGetStackHighWaterMark(NULL));

        esp_err_t ret;

        ret = aht20_read_float(aht20_handle_0, &temp0, &hum0);
        if (ret == ESP_OK)
        {
            ESP_LOGI(TAG, "Sensor 0:\t%.2f °C\t%.2f %%", temp0, hum0);
        }
        else
        {
            ESP_LOGE(TAG, "Error reading sensor 0: %s", esp_err_to_name(ret));
            xEventGroupClearBits(sensor_bits, 0xFF);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        ret = aht20_read_float(aht20_handle_1, &temp1, &hum1);
        if (ret == ESP_OK)
        {
            ESP_LOGI(TAG, "Sensor 1:\t%.2f °C\t%.2f %%", temp1, hum1);
        }
        else
        {
            ESP_LOGE(TAG, "Error reading sensor 1: %s", esp_err_to_name(ret));
            xEventGroupClearBits(sensor_bits, 0xFF);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        EventBits_t bits = BIT_SENSOR_READY;

        if (hum0 > 25)
        {
            bits |= BIT_HUMIDITY_HIGH;
        }

        if (temp1 > 40)
        {
            bits |= BIT_HEATER_HIGH;
        }

        if (temp1 > 80)
        {
            bits |= BIT_HEATER_OVERHEAT;
        }

        xEventGroupSetBits(sensor_bits, bits);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}