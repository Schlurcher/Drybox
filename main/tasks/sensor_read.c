#include "sensor_read.h"
#include "event_bits.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"

// External references
extern EventGroupHandle_t sensor_bits;
extern const char *TAG;

void sensor_read_task(void *pvParameters) {
    float humidity = 0;
    float temperature = 0;

    while (1) {
        const esp_err_t err = ESP_OK;

        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read SHT3x sensor: %s", esp_err_to_name(err));
            xEventGroupClearBits(sensor_bits, 0xFF);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        EventBits_t bits = BIT_SENSOR_READY;

        if (humidity > 25) {
            bits |= BIT_HUMIDITY_HIGH;
        }

        if (temperature > 40) {
            bits |= BIT_HEATER_HIGH;
        }

        if (temperature > 80) {
            bits |= BIT_HEATER_OVERHEAT;
        }

        xEventGroupSetBits(sensor_bits, bits);

        ESP_LOGI(TAG, "Humidity: %.2f%%, Temperature: %.2f°C", humidity, temperature);

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
