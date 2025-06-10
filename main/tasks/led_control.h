//
// Created by Stefan Pohl on 10.06.25.
//

#ifndef LED_CONTROL_H
#define LED_CONTROL_H
#include "led_strip_types.h"

/**
 * @brief Task to monitor and control the LED strip based on system events.
 *
 * This function manages the behavior of the LED strip by waiting for specific
 * event flags from the event group. It activates or deactivates the LED strip
 * based on the system state such as sensor readiness, override conditions,
 * or heater overheating. The LED strip is also configured during initialization
 * to define its GPIO, LED model, transmission color format, and SPI-specific parameters.
 * The task can enable or disable blinking functionality through other tasks,
 * such as the red blinking task.
 *
 * @param params Pointer to the task parameters, typically unused and set to NULL.
 */
void led_control_task(void *params);

/**
 * @brief Task to blink the LED strip with a red color at a fixed interval.
 *
 * This function toggles the LED strip between red and off states in an infinite loop.
 * The red color is achieved by setting the LED strip to maximum red intensity
 * while keeping green and blue intensities at zero. The blinking interval
 * is controlled by a delay between each toggle.
 *
 * @param params Pointer to the task parameters, typically unused and set to NULL.
 */
void led_blink_red_task(void *params);

/**
 * @brief Sets the color of all pixels in the LED strip to the specified RGB values.
 *
 * This function iterates through all the pixels in the LED strip, setting each
 * one to the specified red, green, and blue intensity values.
 *
 * @param handle The handle to the LED strip.
 * @param r The intensity of the red component (0-255).
 * @param g The intensity of the green component (0-255).
 * @param b The intensity of the blue component (0-255).
 */
void led_strip_set_all_pixels(led_strip_handle_t handle, uint8_t r, uint8_t g, uint8_t b);

#endif //LED_CONTROL_H
