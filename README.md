# uros_DalyBMS

This project interfaces a Daly BMS with an ESP32 microcontroller using the ESP-IDF framework and micro-ROS. The project is written in C++ and C, utilizing several components to handle serial communication, GPIO handling, and battery state management.

## Project Structure

The main components of the project include:

root directori
- **main.cpp**: Main entry point of the application.
- **battery_state.cpp/hpp**: Source and header files for managing the battery state.
- **esp32_serial_transport.c/h**: Source and header files for handling serial transport specific to ESP32.
- **gpio_handler.cpp/hpp**: Source and header files for managing GPIO interactions.
- **uros_entities.cpp/hpp**: Source and header files defining entities for micro-ROS integration.
- **CMakeLists.txt**: CMake configuration file to compile the project.
- **component.mk**: Component makefile.
- **Kconfig.projbuild**: Project configuration for ESP-IDF.
- **sdkconfig.defaults**: Default SDK configuration.
- **app-colcon.meta**: Metadata for Colcon build system.

## Configuration

Configuration is handled through the `Kconfig.projbuild` file and during the initialization of the classes and components. Specific parameters such as GPIO numbers and serial configurations are set in the respective initialization functions.

## Usage

To use this firmware, follow these steps:

1. **Install dependencies**: Ensure the ESP-IDF framework and micro-ROS are properly installed and configured.
2. **Add Component**: Use `idf.py prepare` for upload components to `managed_components` directory of your ESP-IDF project.
3. **Configuration**: Use `idf.py menuconfig` to modify "Component - LEDRGB settings" and "Component - DalyBMS GPIO Settings" as needed.
4. **Build**: Use `idf.py build` to compile your project.
5. **Flash**: Use `idf.py flash` to flash the firmware to your ESP32*.
6. **Monitor**: Use `idf.py monitor` to monitor the serial messages sent by the ESP32*.

### Running with ROS 2

To run the micro-ROS agent and echo the topic data, follow these steps:

1. **Run the micro-ROS agent**:
    ```sh
    ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
    ```

2. **Echo the topic data**:
    ```sh
    ros2 topic echo /DalyBMS
    ```

This will allow you to see the data being published by the ESP32 firmware on the `/DalyBMS` topic.


## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Contributions

Contributions are welcome! Please submit a pull request or open an issue to discuss any changes you wish to make.

## Authors

- [Lionel ORCIL](https://github.com/ioio2995)