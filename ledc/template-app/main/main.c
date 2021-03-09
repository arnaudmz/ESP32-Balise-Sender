/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_sleep.h"
#include "soc/rtc.h"

void app_main(void) {
    rtc_clk_slow_freq_set(RTC_SLOW_FREQ_8MD256);
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_8_BIT, // resolution of PWM duty
        .freq_hz = 100,                      // frequency of PWM signal
        .speed_mode = LEDC_LOW_SPEED_MODE,           // timer mode
        .timer_num = LEDC_TIMER_0,            // timer index
        .clk_cfg = LEDC_USE_RTC8M_CLK,              // Auto select the source clock
    };
    // Set configuration of timer0 for high speed channels
    ledc_timer_config(&ledc_timer);
    ledc_channel_config_t ledc_channel = {
            .channel    = LEDC_CHANNEL_0,
            .duty       = 0,
            .gpio_num   = 5,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER_0
        };
    ledc_channel_config(&ledc_channel);
    ESP_ERROR_CHECK( ledc_fade_func_install(0) );
    while (1) {
        printf("up\n");
        ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 255, 50);
        ledc_fade_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, LEDC_FADE_NO_WAIT);
        vTaskDelay(1750 / portTICK_PERIOD_MS);
        //printf("sleep\n");
        //esp_sleep_enable_gpio_wakeup();
        //esp_sleep_enable_timer_wakeup(500000);
        //esp_light_sleep_start();
        printf("down\n");
        ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0, 50);
        ledc_fade_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, LEDC_FADE_NO_WAIT);
        vTaskDelay(1750 / portTICK_PERIOD_MS);
        //printf("sleep\n");
        //esp_sleep_enable_gpio_wakeup();
        //esp_sleep_enable_timer_wakeup(500000);
        //esp_light_sleep_start();
    }
}
