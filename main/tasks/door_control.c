#include "door_control.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "event_bits.h"

// External references
extern EventGroupHandle_t sensor_bits;
extern const char *TAG;

#define GPIO_DOOR       16

void door_control_task(void *params) {
    gpio_set_level(GPIO_DOOR, 0);

    while (1) {
        ESP_LOGI(TAG, "Door task waiting for events...");
        const EventBits_t bits = xEventGroupWaitBits(sensor_bits,
            BIT_SENSOR_READY | BIT_HUMIDITY_HIGH,
            pdFALSE, // Don't clear bits
            pdFALSE, // Any bit will do
            portMAX_DELAY);

        // Failsafe
        if (!(bits & BIT_SENSOR_READY)) {
            gpio_set_level(GPIO_DOOR, 1);
            ESP_LOGW(TAG, "Door CLOSE: Sensor failure");
            continue;
        }

        if (bits & BIT_HUMIDITY_HIGH) {
            gpio_set_level(GPIO_DOOR, 1);
            ESP_LOGI(TAG, "Door OPEN: High humidity");
            continue;
        }

        ESP_LOGI(TAG, "Door CLOSE: Idle");
        gpio_set_level(GPIO_DOOR, 0);
    }
}
