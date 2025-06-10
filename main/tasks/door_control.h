//
// Created on 10.06.25.
//

#ifndef DOOR_CONTROL_H
#define DOOR_CONTROL_H

/**
 * @brief Task to monitor and control the door based on system events.
 *
 * This function manages the behavior of the door by waiting for specific
 * event flags from the event group. It activates or deactivates the door
 * based on system conditions such as sensor readiness and humidity levels.
 * The door is opened in failsafe conditions like sensor failure or when
 * humidity is high. Otherwise, the door remains closed during idle conditions.
 *
 * @param params Pointer to the task parameters, typically unused and set to NULL.
 */
void door_control_task(void *params);

#endif //DOOR_CONTROL_H
