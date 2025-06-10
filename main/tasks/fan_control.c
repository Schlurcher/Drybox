#include "fan_control.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "event_bits.h"

// External references
extern EventGroupHandle_t sensor_bits;
extern const char *TAG;

#define GPIO_FAN        14

void fan_control_task(void *params) {
    gpio_set_level(GPIO_FAN, 0);

    while (1) {
        ESP_LOGI(TAG, "Fan task waiting for events...");
        const EventBits_t bits = xEventGroupWaitBits(sensor_bits,
            BIT_SENSOR_READY | BIT_HUMIDITY_HIGH | BIT_HEATER_OVERHEAT | BIT_HEATER_HIGH,
            pdFALSE, // Don't clear bits
            pdFALSE, // Any bit will do
            portMAX_DELAY);

        // Failsafe
        if (!(bits & BIT_SENSOR_READY) || bits & BIT_HEATER_OVERHEAT) {
            gpio_set_level(GPIO_FAN, 1);
            ESP_LOGW(TAG, "Fan ON: %s", !(bits & BIT_SENSOR_READY) ? "Sensor failure" : "Overheating");
            continue;
        }

        if (bits & (BIT_HUMIDITY_HIGH | BIT_HEATER_HIGH)) {
            gpio_set_level(GPIO_FAN, 1);
            ESP_LOGI(TAG, "Fan ON: %s", (bits & BIT_HUMIDITY_HIGH) ? "High humidity" : "High heater temperature");
            continue;
        }

        gpio_set_level(GPIO_FAN, 0); // Turn off fan
        ESP_LOGI(TAG, "Fan OFF: Idle");
    }
}
