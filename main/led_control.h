//
// Created by Stefan Pohl on 10.06.25.
//

#ifndef LED_CONTROL_H
#define LED_CONTROL_H

// Function to start the LED control task
void led_control_task(void *pvParameters);

// External reference to the LED blink task
void led_blink_red_task(void *pvParameters);

#endif //LED_CONTROL_H
