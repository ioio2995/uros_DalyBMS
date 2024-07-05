idf_component_register(SRCS "main.cpp" "gpio_handler.cpp" "battery_state.cpp" "uros_entities.cpp" "esp32_serial_transport.c"
                        INCLUDE_DIRS "." "../include"
                        REQUIRES dalybms ic_led_rgb micro_ros_espidf_component driver)