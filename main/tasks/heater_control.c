#include "heater_control.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "event_bits.h"

// External references
extern EventGroupHandle_t sensor_bits;
extern const char *TAG;

#define GPIO_HEATER     15

void heater_control_task(void *pvParameters) {
    // Configure LED pin
    const gpio_config_t led_config = {
        .pin_bit_mask = (1ULL << GPIO_HEATER),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&led_config);
    gpio_set_level(GPIO_HEATER, 0);

    while (1) {
        ESP_LOGI(TAG, "Heater task waiting for events...");
        const EventBits_t bits = xEventGroupWaitBits(sensor_bits,
            BIT_SENSOR_READY | BIT_HUMIDITY_HIGH | BIT_HEATER_OVERHEAT,
            pdFALSE, // Don't clear bits
            pdFALSE, // Any bit will do
            portMAX_DELAY);

        // Failsafe
        if (!(bits & BIT_SENSOR_READY) || bits & BIT_HEATER_OVERHEAT) {
            gpio_set_level(GPIO_HEATER, 0);
            ESP_LOGW(TAG, "Heater OFF: %s", !(bits & BIT_SENSOR_READY) ? "Sensor failure" : "Overheating");
            continue;
        }

        if (bits & BIT_HUMIDITY_HIGH) {
            gpio_set_level(GPIO_HEATER, 1);
            ESP_LOGI(TAG, "Heater ON: High humidity");
            continue;
        }

        ESP_LOGI(TAG, "Heater OFF: Idle");
        gpio_set_level(GPIO_HEATER, 0);
    }
}
