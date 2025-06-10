//
// Created on 10.06.25.
//

#ifndef SENSOR_READ_H
#define SENSOR_READ_H

/**
 * @brief Task to read sensor data and set appropriate event bits.
 *
 * This function periodically reads humidity and temperature values from the sensor,
 * and sets appropriate event bits based on the readings. It handles sensor failure
 * scenarios and updates the event group with the current system state.
 *
 * @param pvParameters Pointer to the task parameters, typically unused and set to NULL.
 */
void sensor_read_task(void *pvParameters);

#endif //SENSOR_READ_H
