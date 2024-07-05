#ifndef UROS_ENTITIES_HPP
#define UROS_ENTITIES_HPP

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/battery_state.h>
#include <rmw_microros/rmw_microros.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

#define UROS_CYCLE 10000

bool create_entities();
void destroy_entities();

extern rcl_publisher_t publisher;
extern rcl_allocator_t allocator;
extern rclc_support_t support;
extern rcl_node_t node;
extern rcl_timer_t timer;
extern rclc_executor_t executor;

#endif // UROS_ENTITIES_HPP
