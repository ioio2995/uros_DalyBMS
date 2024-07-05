#ifndef GPIO_HANDLER_HPP
#define GPIO_HANDLER_HPP

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "dalybms.hpp" 

void init_gpio();
void IRAM_ATTR gpio_isr_handler(void* arg);
void gpio_task(void* arg);
void trigger_bms_wakeup(bool activate);

extern DalyBMS bms_handler; 

#endif // GPIO_HANDLER_HPP
