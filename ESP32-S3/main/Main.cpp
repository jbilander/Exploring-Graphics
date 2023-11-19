#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

static uint8_t s_led_state = 0;

static void configure_led(void)
{
    gpio_reset_pin(GPIO_NUM_47);
    gpio_set_direction(GPIO_NUM_47, GPIO_MODE_OUTPUT);
}

extern "C" void app_main()
{
    configure_led();

    while (1)
    {
        gpio_set_level(GPIO_NUM_47, s_led_state);
        s_led_state = !s_led_state;
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}