#ifndef BATTERY_STATE_HPP
#define BATTERY_STATE_HPP

#include <vector>
#include <string>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/battery_state.h>

uint8_t determine_battery_health(const std::vector<std::string>& errors);
void timer_callback(rcl_timer_t * timer, int64_t last_call_time);

void init_watchdog_timer();
void start_watchdog_timer();

extern sensor_msgs__msg__BatteryState battery_state_msg;
extern rcl_publisher_t publisher;

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
};
extern states state;

#endif // BATTERY_STATE_HPP
