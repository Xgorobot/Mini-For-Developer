#include "driver/gpio.h"
#ifndef BLINK_LED_PIN
#define BLINK_LED_PIN 2  
#endif
void led_init()
{
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.21
    io_conf.pin_bit_mask = 1ULL << BLINK_LED_PIN;
    //disable pull-down mode
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    //disable pull-up mode
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
}