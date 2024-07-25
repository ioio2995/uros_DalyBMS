idf_component_register(SRCS "main.cpp" "gpio_handler.cpp" "battery_helpers.cpp" "uros_entities.cpp" "esp32_serial_transport.c"
                        INCLUDE_DIRS "." "../include"
                        REQUIRES ESP_DalyBMS ESP_IC_Led_RGB micro_ros_espidf_component driver)