#include "esp_log.h"
#include "driver/gpio.h"

#include "gpio_handler.hpp"
#include "dalybms.hpp"

#define DEBOUNCE_TIME_MS 200
#define ESP_INTR_FLAG_DEFAULT 0
#define TAG "GPIO"

static QueueHandle_t gpio_evt_queue = NULL;
extern DalyBMS bms_handler;

void IRAM_ATTR gpio_isr_handler(void* arg) {   
    static uint32_t last_isr_time = 0;
    uint32_t current_time = xTaskGetTickCountFromISR();

    if (current_time - last_isr_time > pdMS_TO_TICKS(DEBOUNCE_TIME_MS)) {
         gpio_num_t gpio_num = (gpio_num_t) (int) arg;
        xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
        last_isr_time = current_time;
    }
}

void init_gpio() {
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << gpio_num_t(CONFIG_DALYBMS_STARTUP_PIN));
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(gpio_num_t(CONFIG_DALYBMS_STARTUP_PIN), gpio_isr_handler, (void*) gpio_num_t(CONFIG_DALYBMS_STARTUP_PIN));
    xTaskCreate(gpio_task, "gpio_task", 4096, NULL, 10, NULL);

    gpio_set_direction(gpio_num_t(CONFIG_DALYBMS_WAKEUP_PIN), GPIO_MODE_OUTPUT);
}

void gpio_task(void* arg) {
    uint32_t io_num;
    for (;;) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            ESP_LOGI(TAG, "GPIO %lu interrupt triggered", io_num);
            trigger_bms_wakeup(true);
            vTaskDelay(pdMS_TO_TICKS(2000));
            trigger_bms_wakeup(false);
            vTaskDelay(pdMS_TO_TICKS(500));
            bms_handler.set_discharge_mosfet(true);
        }
    }
}

void trigger_bms_wakeup(bool activate) {
    gpio_set_level(gpio_num_t(CONFIG_DALYBMS_WAKEUP_PIN), activate ? 1 : 0);
}
