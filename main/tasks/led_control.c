#include "led_control.h"
#include "led_strip.h"
#include "esp_log.h"
#include "event_bits.h"

#define GPIO_LED 17

// External references
extern EventGroupHandle_t sensor_bits;

static const char* TAG = "LED_CONTROL";

#define LED_COUNT 80

led_strip_handle_t led_strip;
TaskHandle_t led_blink_red_task_handle = NULL;

void led_blink_red_task(void *params) {
    bool led_on = false;

    while (1) {
        led_on = !led_on;
        if (led_on) {
            led_strip_set_all_pixels(led_strip, 255, 0, 0); // Red ON
        } else {
            led_strip_set_all_pixels(led_strip, 0, 0, 0); // OFF
        }
        led_strip_refresh(led_strip);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void led_strip_set_all_pixels(led_strip_handle_t handle, uint8_t r, uint8_t g, uint8_t b) {
    for (int i = 0; i < LED_COUNT; i++) {
        led_strip_set_pixel(handle, i, r, g, b);
    }
}

void led_control_task(void *params) {
    // Initialization phase
    ESP_LOGI("LED", "Initializing LED control");

    /// LED strip common configuration
    const led_strip_config_t strip_config = {
        .strip_gpio_num = GPIO_LED, // The GPIO that connected to the LED strip's data line
        .max_leds = 1, // The number of LEDs in the strip,
        .led_model = LED_MODEL_WS2812, // LED strip model, it determines the bit timing
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB, // The color component format is G-R-B
        .flags = {
            .invert_out = false, // don't invert the output signal
        }
    };

    /// SPI backend specific configuration
    const led_strip_spi_config_t spi_config = {
        .clk_src = SPI_CLK_SRC_DEFAULT, // different clock source can lead to different power consumption
        .spi_bus = SPI2_HOST, // SPI bus ID
        .flags = {
            .with_dma = true, // Using DMA can improve performance and help drive more LEDs
        }
    };

    /// Create the LED strip object
    ESP_ERROR_CHECK(led_strip_new_spi_device(&strip_config, &spi_config, &led_strip));

    xTaskCreate(led_blink_red_task, "led_blink_red", 512, NULL, 4, &led_blink_red_task_handle);
    vTaskSuspend(led_blink_red_task_handle);

    while (1) {
        const EventBits_t bits = xEventGroupWaitBits(sensor_bits,
            BIT_SENSOR_READY | BIT_LED_OVERRIDE,
            pdFALSE, // Don't clear bits
            pdFALSE, // Any bit will do
            portMAX_DELAY);

        // Turn LED ON to catch attention
        if (!(bits & BIT_SENSOR_READY) || bits & BIT_HEATER_OVERHEAT) {
            vTaskResume(led_blink_red_task_handle);
            ESP_LOGW(TAG, "LED ON: Sensor failure");
            continue;
        }

        vTaskSuspend(led_blink_red_task_handle);

        if (bits & BIT_LED_OVERRIDE) {
            led_strip_set_pixel(led_strip, 0, 255, 255, 255);
            led_strip_refresh(led_strip);
            ESP_LOGI(TAG, "LED ON: Override");
            continue;
        }

        led_strip_set_pixel(led_strip, 0, 0, 0, 0);
        led_strip_refresh(led_strip);
        ESP_LOGI(TAG, "LED OFF: Idle");
    }
}
