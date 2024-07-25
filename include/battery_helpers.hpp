#ifndef BATTERY_HELPERS_HPP
#define BATTERY_HELPERS_HPP

#include <vector>
#include <string>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/battery_state.h>


extern sensor_msgs__msg__BatteryState battery_state_msg;

uint8_t determine_battery_health(const std::vector<std::string>& errors);

#endif // BATTERY_HELPERS_HPP
