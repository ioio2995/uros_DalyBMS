#include "battery_state.hpp"
#include "esp_log.h"
#include "dalybms.hpp"
#include "gpio_handler.hpp"
#include "driver/gptimer.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"
 
#define WATCHDOG_TIMER_GROUP TIMER_GROUP_0
#define WATCHDOG_TIMER TIMER_0
#define WATCHDOG_PULSE_DURATION_MS (CONFIG_DALYBMS_WD_PULSE_DURATION_MS)
#define WATCHDOG_INTERVAL_MINUTES (CONFIG_DALYBMS_WD_INTERVAL_MINUTES)
#define WATCHDOG_INTERVAL_MS (WATCHDOG_INTERVAL_MINUTES * 60 * 1000)

sensor_msgs__msg__BatteryState battery_state_msg;
extern DalyBMS bms_handler;
extern rcl_publisher_t publisher;
extern states state;


static const char* TAG = "BMS_STATE";

uint8_t determine_battery_health(const std::vector<std::string>& errors) {
    if (errors.empty()) {
        return sensor_msgs__msg__BatteryState__POWER_SUPPLY_HEALTH_GOOD;
    }

    for (const auto& error : errors) {
        if (error.find("overheat") != std::string::npos) {
            return sensor_msgs__msg__BatteryState__POWER_SUPPLY_HEALTH_OVERHEAT;
        }
        if (error.find("overvoltage") != std::string::npos) {
            return sensor_msgs__msg__BatteryState__POWER_SUPPLY_HEALTH_OVERVOLTAGE;
        }
        if (error.find("low voltage") != std::string::npos) {
            return sensor_msgs__msg__BatteryState__POWER_SUPPLY_HEALTH_DEAD;
        }
    }

    return sensor_msgs__msg__BatteryState__POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        auto soc = bms_handler.get_soc();
        auto mosfet_status = bms_handler.get_mosfet_status();
        auto status = bms_handler.get_status();
        auto cell_voltages = bms_handler.get_cell_voltages();
        auto temperatures = bms_handler.get_temperatures();
        auto errors = bms_handler.get_errors();

        battery_state_msg.voltage = soc["total_voltage"]/10.0;
        battery_state_msg.current = soc["current"]/10.0;
        battery_state_msg.percentage = soc["soc_percent"]/1000.0;

        if (!rosidl_runtime_c__float__Sequence__init(&battery_state_msg.cell_voltage, cell_voltages.size())) {
            ESP_LOGE(TAG, "Failed to initialize cell_voltage sequence");
            return;
        }
        for (const auto &kv : cell_voltages) {
            battery_state_msg.cell_voltage.data[kv.first - 1] = kv.second/1000.0f;
        }

        if (!rosidl_runtime_c__float__Sequence__init(&battery_state_msg.cell_temperature, temperatures.size())) {
            ESP_LOGE(TAG, "Failed to initialize cell_temperature sequence");
            return;
        }
        float sum_temperature = 0.0;
        for (const auto &kv : temperatures) {
            battery_state_msg.cell_temperature.data[kv.first - 1] = kv.second;
            sum_temperature += kv.second;
        }
        float average_temperature = temperatures.size() > 0 ? sum_temperature / temperatures.size() : 0.0;
        battery_state_msg.temperature = average_temperature;

        battery_state_msg.power_supply_status = mosfet_status["charging_mosfet"]
                ? sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_CHARGING
                : sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_DISCHARGING;

        battery_state_msg.power_supply_technology = sensor_msgs__msg__BatteryState__POWER_SUPPLY_TECHNOLOGY_LION;
        battery_state_msg.power_supply_health = determine_battery_health(errors);
        
        rcl_ret_t ret = rcl_publish(&publisher, &battery_state_msg, NULL);
        if (ret != RCL_RET_OK) {
            printf("Failed to publish message. Status: %d\n", (int)ret);
            state = AGENT_DISCONNECTED;
        }
        rosidl_runtime_c__float__Sequence__fini(&battery_state_msg.cell_voltage);
        rosidl_runtime_c__float__Sequence__fini(&battery_state_msg.cell_temperature);
    }
}


static gptimer_handle_t timer = NULL;
static bool is_pulsing = false;

bool IRAM_ATTR watchdog_isr(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx) {
    

    if (!is_pulsing) {
        // Start the pulse
        trigger_bms_wakeup(true);
        gptimer_alarm_config_t alarm_config = {
            .alarm_count = WATCHDOG_PULSE_DURATION_MS * 1000,
            .reload_count = 0,
            .flags = {
                .auto_reload_on_alarm = false,
            }
        };
        gptimer_set_alarm_action(timer, &alarm_config);
        is_pulsing = true;
    } else {
        // End the pulse
        trigger_bms_wakeup(false);
        gptimer_alarm_config_t alarm_config = {
            .alarm_count = WATCHDOG_INTERVAL_MS * 1000,
            .reload_count = 0,
            .flags = {
                .auto_reload_on_alarm = true,
            }
        };
        gptimer_set_alarm_action(timer, &alarm_config);
        is_pulsing = false;
    }

    return true; // Return true to yield higher priority task
}


void init_watchdog_timer() {
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_APB,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1MHz, 1 tick = 1us
    };

    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &timer));

    gptimer_event_callbacks_t cbs = {
        .on_alarm = watchdog_isr,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(timer, &cbs, NULL));

    gptimer_alarm_config_t alarm_config = {
        .alarm_count = WATCHDOG_INTERVAL_MS * 1000,
        .reload_count = 0,
        .flags = {
            .auto_reload_on_alarm = true,
        }
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(timer, &alarm_config));

    ESP_ERROR_CHECK(gptimer_enable(timer));

    ESP_LOGI(TAG, "Watchdog timer initialized");
}

void start_watchdog_timer() {
    // Trigger initial pulse
    trigger_bms_wakeup(true);
    vTaskDelay(pdMS_TO_TICKS(WATCHDOG_PULSE_DURATION_MS));
    trigger_bms_wakeup(false);

    ESP_ERROR_CHECK(gptimer_start(timer));
}