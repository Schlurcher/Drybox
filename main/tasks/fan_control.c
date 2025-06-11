#include "fan_control.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "event_bits.h"

static const char* TAG = "FAN_CONTROL";

extern EventGroupHandle_t sensor_bits;

void fan_control_task(void* params)
{
    ESP_LOGI(TAG, "Started on Core: %d, Stack: %d", xPortGetCoreID(), uxTaskGetStackHighWaterMark(NULL));
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP_LOGI(TAG, "Core: %d, Stack: %d", xPortGetCoreID(), uxTaskGetStackHighWaterMark(NULL));

        const EventBits_t bits = xEventGroupWaitBits(sensor_bits,
            BIT_SENSOR_READY | BIT_HUMIDITY_HIGH,
            pdFALSE, // Don't clear bits
            pdFALSE, // Any bit will do
            portMAX_DELAY);

        // Failsafe
        if (!(bits & BIT_SENSOR_READY)) {
            ESP_LOGW(TAG, "Door CLOSE: Sensor failure");
            continue;
        }

        if (bits & BIT_HUMIDITY_HIGH) {
            ESP_LOGI(TAG, "Door OPEN: High humidity");
            continue;
        }

        ESP_LOGI(TAG, "Door CLOSE: Idle");
    }
}
