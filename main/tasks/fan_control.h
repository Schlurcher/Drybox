//
// Created on 10.06.25.
//

#ifndef FAN_CONTROL_H
#define FAN_CONTROL_H

/**
 * @brief Task to monitor and control the fan based on system events.
 *
 * This function manages the behavior of the fan by waiting for specific
 * event flags from the event group. It activates or deactivates the fan
 * based on system conditions such as sensor readiness, humidity levels,
 * and heater temperature. The fan is turned on in failsafe conditions
 * like sensor failure or heater overheating, or when humidity or heater
 * temperature is high. Otherwise, the fan remains off during idle conditions.
 *
 * @param params Pointer to the task parameters, typically unused and set to NULL.
 */
void fan_control_task(void *params);

#endif //FAN_CONTROL_H
