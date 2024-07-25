#ifndef GPIO_HANDLER_HPP
#define GPIO_HANDLER_HPP

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void IRAM_ATTR gpio_isr_handler(void* arg);
void init_gpio();
void gpio_task(void* arg);
void trigger_bms_wakeup(bool activate);

#endif // GPIO_HANDLER_HPP
