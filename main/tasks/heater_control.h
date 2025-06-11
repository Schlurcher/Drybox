//
// Created on 10.06.25.
//

#ifndef HEATER_CONTROL_H
#define HEATER_CONTROL_H

/**
 * @brief Task to monitor and control the heater based on system events.
 *
 * This function manages the behavior of the heater by waiting for specific
 * event flags from the event group. It activates or deactivates the heater
 * based on system conditions such as sensor readiness, humidity levels,
 * and overheating. The heater is turned off in failsafe conditions
 * like sensor failure or overheating. Otherwise, it turns on when humidity is high.
 *
 * @param params Pointer to the task parameters, typically unused and set to NULL.
 */
void heater_control_task(void *params);

#endif //HEATER_CONTROL_H
