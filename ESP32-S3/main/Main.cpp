#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_lcd_panel_rgb.h"

static const char *TAG = "RGB_LCD";
static uint8_t s_led_state = 0;

#define LCD_PIXEL_CLOCK_HZ (18 * 1000 * 1000)
#define PIN_NUM_HSYNC 1   // New Line
#define PIN_NUM_VSYNC 2   // New Frame
#define PIN_NUM_DE 48     // Data Enable On/Off
#define PIN_NUM_PCLK 47   // Pixel clock
#define PIN_NUM_DATA0 15  // B0
#define PIN_NUM_DATA1 16  // B1
#define PIN_NUM_DATA2 17  // B2
#define PIN_NUM_DATA3 18  // B3
#define PIN_NUM_DATA4 21  // B4
#define PIN_NUM_DATA5 9   // G0
#define PIN_NUM_DATA6 10  // G1
#define PIN_NUM_DATA7 11  // G2
#define PIN_NUM_DATA8 12  // G3
#define PIN_NUM_DATA9 13  // G4
#define PIN_NUM_DATA10 14 // G5
#define PIN_NUM_DATA11 4  // R0
#define PIN_NUM_DATA12 5  // R1
#define PIN_NUM_DATA13 6  // R2
#define PIN_NUM_DATA14 7  // R3
#define PIN_NUM_DATA15 8  // R4

// The pixel number in horizontal and vertical
#define LCD_H_RES 800
#define LCD_V_RES 480

// FrameBuffer
#define LCD_NUM_FB 2 // allocate double frame buffer, Maximum number of buffers are 3

static void configure_led(void)
{
    gpio_reset_pin(GPIO_NUM_38);
    gpio_set_direction(GPIO_NUM_38, GPIO_MODE_OUTPUT);
}

static void configure_rgb_lcd(void)
{
    ESP_LOGI(TAG, "Install RGB LCD panel driver");
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_rgb_panel_config_t panel_config = {
        .clk_src = LCD_CLK_SRC_DEFAULT,
        .timings = {// The timing parameters should refer to your LCD spec
                    .pclk_hz = LCD_PIXEL_CLOCK_HZ,
                    .h_res = LCD_H_RES,
                    .v_res = LCD_V_RES,
                    .hsync_pulse_width = 1,
                    .hsync_back_porch = 40,
                    .hsync_front_porch = 20,
                    .vsync_pulse_width = 1,
                    .vsync_back_porch = 8,
                    .vsync_front_porch = 4},
        .data_width = 16, // RGB565 in parallel mode, thus 16-bit in width
        .num_fbs = LCD_NUM_FB,
        .hsync_gpio_num = PIN_NUM_HSYNC,
        .vsync_gpio_num = PIN_NUM_VSYNC,
        .de_gpio_num = PIN_NUM_DE,
        .pclk_gpio_num = PIN_NUM_PCLK,
        .disp_gpio_num = -1, // display control signal not used
        .data_gpio_nums = {PIN_NUM_DATA0, PIN_NUM_DATA1, PIN_NUM_DATA2, PIN_NUM_DATA3, PIN_NUM_DATA4, PIN_NUM_DATA5, PIN_NUM_DATA6, PIN_NUM_DATA7, PIN_NUM_DATA8, PIN_NUM_DATA9, PIN_NUM_DATA10, PIN_NUM_DATA11, PIN_NUM_DATA12, PIN_NUM_DATA13, PIN_NUM_DATA14, PIN_NUM_DATA15},
        .flags = {.fb_in_psram = true}};
    ESP_ERROR_CHECK(esp_lcd_new_rgb_panel(&panel_config, &panel_handle));
}

extern "C" void app_main()
{
    configure_rgb_lcd();
    configure_led();

    while (1)
    {
        gpio_set_level(GPIO_NUM_38, s_led_state);
        s_led_state = !s_led_state;
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}