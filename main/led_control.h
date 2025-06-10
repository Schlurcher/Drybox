//
// Created by Stefan Pohl on 10.06.25.
//

#ifndef LED_CONTROL_H
#define LED_CONTROL_H
#include "led_strip_types.h"

// Function to start the LED control task
void led_control_task(void *pvParameters);

// External reference to the LED blink task
void led_blink_red_task(void *pvParameters);


void led_strip_set_all_pixels(led_strip_handle_t handle, uint8_t r, uint8_t g, uint8_t b);

#endif //LED_CONTROL_H
