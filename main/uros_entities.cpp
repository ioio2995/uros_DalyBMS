#include "uros_entities.hpp"
#include "battery_helpers.hpp"
#include "common_defs.h"

#include "dalybms.hpp" 
#include "gpio_handler.hpp"

#include "esp_log.h"

#include "rosidl_runtime_c/primitives_sequence_functions.h"
#include "micro_ros_utilities/type_utilities.h"

#include "sensor_msgs/msg/battery_state.h"
#include "dalybms_interfaces/srv/get_battery_state.h"

#define BATTERY_TECHNOLOGY sensor_msgs__msg__BatteryState__POWER_SUPPLY_TECHNOLOGY_LION

static const char* TAG = "uRos_Entities";

DalyBMS bms_handler;

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_service_t service;
rclc_executor_t executor;

dalybms_interfaces__srv__GetBatteryState_Response response_msg;
dalybms_interfaces__srv__GetBatteryState_Request request_msg;

void service_callback(const void * request, void * response) {

    auto req = static_cast<const dalybms_interfaces__srv__GetBatteryState_Request *>(request);
    auto res = static_cast<dalybms_interfaces__srv__GetBatteryState_Response *>(response);
    (void)req;

    sensor_msgs__msg__BatteryState battery_state_msg;

    trigger_bms_wakeup(true);
    vTaskDelay(pdMS_TO_TICKS(2000));
    trigger_bms_wakeup(false);
    vTaskDelay(pdMS_TO_TICKS(500));
    
    auto soc = bms_handler.get_soc();
    auto mosfet_status = bms_handler.get_mosfet_status();
    auto status = bms_handler.get_status();
    auto cell_voltages = bms_handler.get_cell_voltages();
    auto temperatures = bms_handler.get_temperatures();
    auto errors = bms_handler.get_errors();

    battery_state_msg.voltage = soc["total_voltage"]/10.0;
    battery_state_msg.current = soc["current"]/10.0;
    battery_state_msg.percentage = soc["soc_percent"]/1000.0;

    battery_state_msg.capacity = status["capacity_ah"];

    ESP_LOGD(TAG, "Initialize cell_voltage sequence ");
    if (!rosidl_runtime_c__float__Sequence__init(&battery_state_msg.cell_voltage, cell_voltages.size())) {
        ESP_LOGE(TAG, "Failed to initialize cell_voltage sequence");
        return;
    }
    for (const auto &kv : cell_voltages) {
        if (kv.first - 1 < battery_state_msg.cell_voltage.size) {
            battery_state_msg.cell_voltage.data[kv.first - 1] = kv.second/1000.0f;
        } else {
            ESP_LOGE(TAG, "Cell voltage index out of range");
        }
    }

    ESP_LOGD(TAG, "Initialize cell_temp sequence ");
    if (!rosidl_runtime_c__float__Sequence__init(&battery_state_msg.cell_temperature, temperatures.size())) {
        ESP_LOGE(TAG, "Failed to initialize cell_temperature sequence");
        rosidl_runtime_c__float__Sequence__fini(&battery_state_msg.cell_voltage);
        return;
    }
    float sum_temperature = 0.0;
    for (const auto &kv : temperatures) {
        if (kv.first - 1 < battery_state_msg.cell_temperature.size) {
            battery_state_msg.cell_temperature.data[kv.first - 1] = kv.second;
            sum_temperature += kv.second;
        } else {
            ESP_LOGE(TAG, "Temperature index out of range");
        }
    }
    float average_temperature = temperatures.size() > 0 ? sum_temperature / temperatures.size() : 0.0;
    battery_state_msg.temperature = average_temperature;

    if (mosfet_status["bms_state"] == 0) {
        battery_state_msg.power_supply_status = sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_NOT_CHARGING;
    } else if (mosfet_status["bms_state"] == 1) {
        battery_state_msg.power_supply_status = sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_CHARGING;
    } else if (mosfet_status["bms_state"] == 2) {
        battery_state_msg.power_supply_status = sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_DISCHARGING;
    } else {
        ESP_LOGE(TAG, "Invalid BMS state");
    }

    battery_state_msg.power_supply_technology = BATTERY_TECHNOLOGY;
    battery_state_msg.present = true;
    battery_state_msg.power_supply_health = determine_battery_health(errors);
    ESP_LOGD(TAG, "Initialize cell_voltage sequence in response");
    if (!rosidl_runtime_c__float__Sequence__init(&res->battery_state.cell_voltage, battery_state_msg.cell_voltage.size)) {
        ESP_LOGE(TAG, "Failed to initialize cell_voltage sequence in response");
        rosidl_runtime_c__float__Sequence__fini(&battery_state_msg.cell_voltage);
        return;
    }
    ESP_LOGD(TAG, "Initialize cell_Temp sequence in response");
    if (!rosidl_runtime_c__float__Sequence__init(&res->battery_state.cell_temperature, battery_state_msg.cell_temperature.size)) {
        ESP_LOGE(TAG, "Failed to initialize cell_temperature sequence in response");
        rosidl_runtime_c__float__Sequence__fini(&battery_state_msg.cell_voltage);
        rosidl_runtime_c__float__Sequence__fini(&battery_state_msg.cell_temperature);
        rosidl_runtime_c__float__Sequence__fini(&res->battery_state.cell_voltage);
        return;
    }
    ESP_LOGD(TAG, "Copy sequence in response");
    // Copier les valeurs des séquences
    for (size_t i = 0; i < battery_state_msg.cell_voltage.size; i++) {
        res->battery_state.cell_voltage.data[i] = battery_state_msg.cell_voltage.data[i];
    }
    for (size_t i = 0; i < battery_state_msg.cell_temperature.size; i++) {
        res->battery_state.cell_temperature.data[i] = battery_state_msg.cell_temperature.data[i];
    }
    ESP_LOGD(TAG, "Copy ALL sequence in response");

    res->battery_state.voltage = battery_state_msg.voltage;
    res->battery_state.current = battery_state_msg.current;
    res->battery_state.percentage = battery_state_msg.percentage;
    res->battery_state.capacity = battery_state_msg.capacity;
    res->battery_state.temperature = battery_state_msg.temperature;
    res->battery_state.power_supply_status = battery_state_msg.power_supply_status;
    res->battery_state.power_supply_technology = battery_state_msg.power_supply_technology;
    res->battery_state.present = battery_state_msg.present;
    res->battery_state.power_supply_health = battery_state_msg.power_supply_health;

    rosidl_runtime_c__float__Sequence__fini(&battery_state_msg.cell_voltage);
    rosidl_runtime_c__float__Sequence__fini(&battery_state_msg.cell_temperature);
    ESP_LOGD(TAG, "End");
}

bool create_entities() {
    allocator = rcl_get_default_allocator();

    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "DalyBMS", "", &support));
    RCCHECK(rclc_service_init_default(
        &service,
        &node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(dalybms_interfaces, srv, GetBatteryState),
        "DalyBMS_Service"));
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));


    static micro_ros_utilities_memory_conf_t conf = {0};
    conf.max_string_capacity = 50;
    conf.max_ros2_type_sequence_capacity = 14;
    conf.max_basic_type_sequence_capacity = 14;


    if (!micro_ros_utilities_create_message_memory(
            ROSIDL_GET_MSG_TYPE_SUPPORT(dalybms_interfaces, srv, GetBatteryState_Request), 
            &request_msg, 
            conf)) {
        ESP_LOGE(TAG, "Failed to create message memory for request");
        return false;
    }
    if (!micro_ros_utilities_create_message_memory(
            ROSIDL_GET_MSG_TYPE_SUPPORT(dalybms_interfaces, srv, GetBatteryState_Response), 
            &response_msg, 
            conf)) {
        ESP_LOGE(TAG, "Failed to create message memory for response");
        micro_ros_utilities_destroy_message_memory(
            ROSIDL_GET_MSG_TYPE_SUPPORT(dalybms_interfaces, srv, GetBatteryState_Request), 
            &request_msg, 
            conf);
        return false;
    }

    RCCHECK(rclc_executor_add_service(
        &executor,
        &service,
        &request_msg,
        &response_msg,
        service_callback
        )
    );
    return true;
}
void destroy_entities(const char* error_message) {
    ESP_LOGE(TAG,"%s", error_message);
    ESP_LOGE(TAG,"System Restart!");
    esp_restart();
}