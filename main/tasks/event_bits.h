//
// Created on 10.06.25.
//

#ifndef EVENT_BITS_H
#define EVENT_BITS_H

/**
 * @brief Event bit definitions for the sensor_bits event group
 *
 * These bit definitions are used across all tasks to communicate system state
 * through the shared event group.
 */

// Sensor status bits
#define BIT_SENSOR_READY        (1 << 0)   /**< Indicates that sensor readings are valid */
#define BIT_HUMIDITY_HIGH       (1 << 1)   /**< Indicates high humidity level detected */
#define BIT_HEATER_OVERHEAT     (1 << 2)   /**< Indicates heater temperature is critically high */
#define BIT_HEATER_HIGH         (1 << 3)   /**< Indicates heater temperature is high but not critical */
#define BIT_LED_OVERRIDE        (1 << 4)   /**< Indicates LED should be in override mode */

#endif //EVENT_BITS_H
