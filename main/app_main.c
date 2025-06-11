#include <stdio.h>

#include "esp_lcd_io_i2c.h"
#include "esp_lcd_panel_dev.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_ssd1306.h"
#include "esp_lcd_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_lvgl_port.h"
#include "sdkconfig.h"
#include "driver/i2c_master.h"
#include "driver/i2c_types.h"
#include "tasks/event_bits.h"
#include "tasks/led_control.h"
#include "tasks/fan_control.h"
#include "tasks/door_control.h"
#include "tasks/heater_control.h"
#include "tasks/sensor_read.h"

#define I2C_MASTER_0_SCL_IO 22      /*!< gpio number for I2C master clock */
#define I2C_MASTER_0_SDA_IO 21      /*!< gpio number for I2C master data  */
#define I2C_MASTER_0_NUM I2C_NUM_0  /*!< I2C port number for master dev */

#define I2C_MASTER_1_SCL_IO 32      /*!< gpio number for I2C master clock */
#define I2C_MASTER_1_SDA_IO 33      /*!< gpio number for I2C master data  */
#define I2C_MASTER_1_NUM I2C_NUM_1  /*!< I2C port number for master dev */

#define GPIO_FAN        14
#define GPIO_HEATER     15
#define GPIO_DOOR       16

#define EXAMPLE_LCD_PIXEL_CLOCK_HZ    (400 * 1000)
#define EXAMPLE_PIN_NUM_SDA           18
#define EXAMPLE_PIN_NUM_SCL           23
#define EXAMPLE_PIN_NUM_RST           (-1)
#define EXAMPLE_I2C_HW_ADDR           0x3C

// The pixel number in horizontal and vertical
#define EXAMPLE_LCD_H_RES              128
#define EXAMPLE_LCD_V_RES              64

// Bit number used to represent command and parameter
#define EXAMPLE_LCD_CMD_BITS           8
#define EXAMPLE_LCD_PARAM_BITS         8

// Create an event group handle
EventGroupHandle_t sensor_bits;

static const char* TAG = "DRYBOX";

void i2c_bus_init(const i2c_port_num_t port_num, const int scl_pin, const int sda_pin,
                  i2c_master_bus_handle_t* bus_handle)
{
    const i2c_master_bus_config_t config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = port_num,
        .scl_io_num = scl_pin,
        .sda_io_num = sda_pin,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&config, bus_handle));
    ESP_LOGI(TAG, "I2C bus %d initialized", port_num);
}

i2c_master_bus_handle_t i2c_bus_0;
i2c_master_bus_handle_t i2c_bus_1;

static esp_lcd_panel_handle_t lcd_panel = NULL;
static esp_lcd_panel_io_handle_t io_handle = NULL;
static lv_display_t* lvgl_disp = NULL;

void lcd_init(void)
{
    ESP_LOGI(TAG, "Install panel IO");
    const esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = EXAMPLE_I2C_HW_ADDR,
        .scl_speed_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
        .control_phase_bytes = 1, // According to SSD1306 datasheet
        .lcd_cmd_bits = EXAMPLE_LCD_CMD_BITS, // According to SSD1306 datasheet
        .lcd_param_bits = EXAMPLE_LCD_PARAM_BITS, // According to SSD1306 datasheet
        .dc_bit_offset = 6, // According to SSD1306 datasheet
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus_0, &io_config, &io_handle));

    esp_lcd_panel_dev_config_t panel_config = {
        .bits_per_pixel = 1,
        .reset_gpio_num = EXAMPLE_PIN_NUM_RST
    };
    esp_lcd_panel_ssd1306_config_t ssd1306_config = {
        .height = EXAMPLE_LCD_V_RES,
    };
    panel_config.vendor_config = &ssd1306_config;
    ESP_LOGI(TAG, "Install SSD1306 panel driver");
    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(io_handle, &panel_config, &lcd_panel));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(lcd_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(lcd_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(lcd_panel, true));
}

void lvgl_init()
{
    ESP_LOGI(TAG, "Initialize LVGL");
    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    lvgl_port_init(&lvgl_cfg);

    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io_handle,
        .panel_handle = lcd_panel,
        .buffer_size = EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES,
        .double_buffer = true,
        .hres = EXAMPLE_LCD_H_RES,
        .vres = EXAMPLE_LCD_V_RES,
        .color_format = LV_COLOR_FORMAT_RGB565,
        .monochrome = true,
        .rotation = {
            .swap_xy = false,
            .mirror_x = true,
            .mirror_y = true,
        },
        .flags = {
            .swap_bytes = false,
            .sw_rotate = false,
        }
    };

    lvgl_disp = lvgl_port_add_disp(&disp_cfg);
}

lv_obj_t* table;

void sensor_screen_init(void)
{
    lv_obj_t* scr = lv_scr_act();

    lv_lock();

    table = lv_table_create(scr);

    // Remove padding
    lv_obj_set_style_pad_all(table, 0, LV_PART_ITEMS);

    // Set up table structure
    lv_table_set_col_cnt(table, 3);
    lv_table_set_row_cnt(table, 2);

    // Get display width
    uint32_t disp_width = lv_display_get_physical_horizontal_resolution(NULL);

    // Set column widths: last two columns 50px each, first takes remaining space
    lv_table_set_col_width(table, 1, 50); // Second column
    lv_table_set_col_width(table, 2, 50); // Third column
    lv_table_set_col_width(table, 0, disp_width - 100); // First column (total - other columns)

    lv_table_set_cell_value(table, 0, 0, "S1");
    lv_table_set_cell_value(table, 1, 0, "S2");

    // Make table use full width and height
    lv_obj_set_width(table, disp_width);
    lv_obj_set_height(table, lv_display_get_physical_vertical_resolution(NULL));

    // Align to top-left instead of center
    lv_obj_center(table);

    lv_unlock();
}

void sensor_screen_update(float temp0, float temp1, float hum0, float hum1)
{
    char buffer[16]; // Buffer for string conversion

    lv_lock();

    // Fill table data
    snprintf(buffer, sizeof(buffer), "%2.1f °C", temp0);
    lv_table_set_cell_value(table, 0, 1, buffer);
    snprintf(buffer, sizeof(buffer), "%2.1f °C", temp1);
    lv_table_set_cell_value(table, 0, 2, buffer);

    snprintf(buffer, sizeof(buffer), "%2.1f %%", hum0);
    lv_table_set_cell_value(table, 1, 1, buffer);
    snprintf(buffer, sizeof(buffer), "%2.1f %%", hum1);
    lv_table_set_cell_value(table, 1, 2, buffer);

    lv_unlock();
}

void app_main(void)
{
    // Create the event group before starting tasks
    sensor_bits = xEventGroupCreate();
    ESP_LOGI(TAG, "Initialized sensor_bits");

    i2c_bus_init(I2C_MASTER_0_NUM, I2C_MASTER_0_SCL_IO, I2C_MASTER_0_SDA_IO, &i2c_bus_0);
    i2c_bus_init(I2C_MASTER_1_NUM, I2C_MASTER_1_SCL_IO, I2C_MASTER_1_SDA_IO, &i2c_bus_1);

    lcd_init();
    lvgl_init();

    xTaskCreatePinnedToCore(sensor_read_task, "sensor_read_task", 8192, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(heater_control_task, "heater_control_task", 8192, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(fan_control_task, "fan_control_task", 8192, NULL, 5, NULL, 1);
    // xTaskCreatePinnedToCore(door_control_task, "door_control_task", 1024, NULL, 2, NULL, 1);
    // xTaskCreatePinnedToCore(led_control_task, "led_control_task", 1024, NULL, 5, NULL, 1);
}
