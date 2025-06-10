# Register the main component
idf_component_register(SRCS "app_main.c"
       "tasks/fan_control.c"
       "tasks/door_control.c"
       "tasks/led_control.c"
       "tasks/heater_control.c"
       "tasks/sensor_read.c"
       INCLUDE_DIRS "." "tasks")
