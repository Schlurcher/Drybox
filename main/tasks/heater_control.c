#include "heater_control.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char* TAG = "HEATER_CONTROL";


void heater_control_task(void* params)
{
    ESP_LOGI(TAG, "Started on Core: %d, Stack: %d", xPortGetCoreID(), uxTaskGetStackHighWaterMark(NULL));
    while (1)
    {
        ESP_LOGI(TAG, "Core: %d, Stack: %d", xPortGetCoreID(), uxTaskGetStackHighWaterMark(NULL));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
