#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "sht3x.h"
#include "led_control.h"

#define I2C_MASTER_SCL_IO          GPIO_NUM_2              /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO          GPIO_NUM_1              /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM             I2C_NUM_0              /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ         100000                 /*!< I2C master clock frequency */

#define GPIO_FAN        14
#define GPIO_HEATER     15
#define GPIO_DOOR       16

#define BIT_SENSOR_READY        (1 << 0)
#define BIT_HUMIDITY_HIGH       (1 << 1)
#define BIT_HEATER_OVERHEAT     (1 << 2)
#define BIT_HEATER_HIGH         (1 << 3)
#define BIT_LED_OVERRIDE        (1 << 4)


// Create an event group handle
EventGroupHandle_t sensor_bits;

const char *TAG = "HUMIDITY_CONTROLLER";

static i2c_bus_handle_t i2c_bus = NULL;
static sht3x_handle_t sht3x = NULL;

void init_i2c() {
    const i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_bus = i2c_bus_create(I2C_MASTER_NUM, &conf);
}

void init_sht3x() {
    sht3x = sht3x_create(i2c_bus, SHT3x_ADDR_PIN_SELECT_VSS);
    sht3x_set_measure_mode(sht3x, SHT3x_PER_2_MEDIUM);
}

void sensor_read_task(void *pvParameters) {
    float humidity = 0;
    float temperature = 0;

    while (1) {
        const esp_err_t err = sht3x_get_humiture(sht3x, &temperature, &humidity);

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

void fan_control_task(void *pvParameters) {
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

void door_control_task(void *pvParameters) {
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

        ESP_LOGI(TAG, "Heater CLOSE: Idle");
        gpio_set_level(GPIO_DOOR, 0);
    }
}

void app_main(void) {
    // Create the event group before starting tasks
    sensor_bits = xEventGroupCreate();

    init_i2c();
    init_sht3x();

    xTaskCreatePinnedToCore(sensor_read_task, "sensor_read_task", 2048, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(heater_control_task, "heater_control_task", 1024, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(fan_control_task, "fan_control_task", 1024, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(door_control_task, "door_control_task", 1024, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(led_control_task, "led_control_task", 1024, NULL, 5, NULL, 1);
}
