#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <cmath>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "uros_entities.hpp"
#include "gpio_handler.hpp"
#include "battery_state.hpp"
#include "ic_led_rgb.hpp"
#include "dalybms.hpp"
#include "esp32_serial_transport.h"
#include <rmw_microros/rmw_microros.h>


static uint8_t led_strip_pixels[CONFIG_LED_NUMBERS * 3];

IC_Led_RGB LRGB(gpio_num_t(CONFIG_LED_STRIP_GPIO_NUM), CONFIG_LED_STRIP_RESOLUTION_HZ);
DalyBMS bms_handler;

uint32_t red;
uint32_t green;
uint32_t blue;

states state;

void micro_ros_task(void * arg)
{
    while (1) {
        switch (state) {
            case WAITING_AGENT:
                state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;
                vTaskDelay(pdMS_TO_TICKS(1000));
                break;
            case AGENT_AVAILABLE:
                state = create_entities() ? AGENT_CONNECTED : WAITING_AGENT;
                if (state == WAITING_AGENT) {
                    destroy_entities();
                }
                break;
            case AGENT_CONNECTED: {
				rcl_ret_t ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
				if (ret != RCL_RET_OK && ret != RCL_RET_TIMEOUT) {
                    printf("Executor spin failed. Status: %d\n", (int)ret);
					state = AGENT_DISCONNECTED;
				};
                break;
            }
            case AGENT_DISCONNECTED:
                destroy_entities();
                state = WAITING_AGENT;
                break;
            default:
                break;
        }

        if (state == AGENT_CONNECTED) {
            red = 0;
            green = 50;
            blue = 50;
        } else {
            red = 50;
            green = 50;
            blue = 0;
        }
        led_strip_pixels[0] = red;
        led_strip_pixels[1] = green;
        led_strip_pixels[2] = blue;
        LRGB.set(led_strip_pixels, sizeof(led_strip_pixels));
    }
    vTaskDelete(NULL);
}

extern "C" void app_main(void)
{
	red = 50;
    green = 0;
    blue = 0;
	led_strip_pixels[0] = red;
	led_strip_pixels[1] = green;
	led_strip_pixels[2] = blue;
	LRGB.set(led_strip_pixels, sizeof(led_strip_pixels));
	state = WAITING_AGENT;
    init_gpio();
    init_watchdog_timer();
    start_watchdog_timer();
    vTaskDelay(pdMS_TO_TICKS(5000));

#if defined(RMW_UXRCE_TRANSPORT_CUSTOM)
	static size_t uart_port = UART_NUM_0;
	rmw_uros_set_custom_transport(
		true,
		(void *) &uart_port,
		esp32_serial_open,
		esp32_serial_close,
		esp32_serial_write,
		esp32_serial_read
	);
#else
#error micro-ROS transports misconfigured
#endif  // RMW_UXRCE_TRANSPORT_CUSTOM
    xTaskCreate(micro_ros_task,
            "uros_task",
            CONFIG_MICRO_ROS_APP_STACK,
            NULL,
            CONFIG_MICRO_ROS_APP_TASK_PRIO,
            NULL);
}