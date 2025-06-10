#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "tasks/event_bits.h"
#include "tasks/led_control.h"
#include "tasks/fan_control.h"
#include "tasks/door_control.h"
#include "tasks/heater_control.h"
#include "tasks/sensor_read.h"

#define I2C_MASTER_SCL_IO          GPIO_NUM_2              /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO          GPIO_NUM_1              /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM             I2C_NUM_0              /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ         100000                 /*!< I2C master clock frequency */

#define GPIO_FAN        14
#define GPIO_HEATER     15
#define GPIO_DOOR       16

// Create an event group handle
EventGroupHandle_t sensor_bits;

const char *TAG = "HUMIDITY_CONTROLLER";

void app_main(void) {
    // Create the event group before starting tasks
    sensor_bits = xEventGroupCreate();

    xTaskCreatePinnedToCore(sensor_read_task, "sensor_read_task", 2048, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(heater_control_task, "heater_control_task", 1024, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(fan_control_task, "fan_control_task", 1024, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(door_control_task, "door_control_task", 1024, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(led_control_task, "led_control_task", 1024, NULL, 5, NULL, 1);
}
