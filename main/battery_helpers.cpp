#include "battery_helpers.hpp"
 
uint8_t determine_battery_health(const std::vector<std::string>& errors) {
    if (errors.empty()) {
        return sensor_msgs__msg__BatteryState__POWER_SUPPLY_HEALTH_GOOD;
    }

    for (const auto& error : errors) {
        if (error.find("temp high") != std::string::npos) {
            return sensor_msgs__msg__BatteryState__POWER_SUPPLY_HEALTH_OVERHEAT;
        }
        if (error.find("SOC high level") != std::string::npos || 
            error.find("Sum volt high level") != std::string::npos || 
            error.find("Cell volt high level") != std::string::npos) {
            return sensor_msgs__msg__BatteryState__POWER_SUPPLY_HEALTH_OVERVOLTAGE;
        }
        if (error.find("SOC low level") != std::string::npos || 
            error.find("Sum volt low level") != std::string::npos || 
            error.find("Cell volt low level") != std::string::npos) {
            return sensor_msgs__msg__BatteryState__POWER_SUPPLY_HEALTH_DEAD;
        }
    }

    return sensor_msgs__msg__BatteryState__POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
}