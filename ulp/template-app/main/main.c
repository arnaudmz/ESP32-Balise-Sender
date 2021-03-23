/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp32/ulp.h"
#include "ulp_main.h"
#include "driver/rtc_io.h"
#include "driver/uart.h"
#include "esp_sleep.h"

extern uint32_t ulp_cmd;

extern const uint8_t bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t bin_end[]   asm("_binary_ulp_main_bin_end");

void ulp_init() {
    ESP_ERROR_CHECK( ulp_load_binary(0, bin_start,(bin_end - bin_start) / sizeof(uint32_t)) );
}

void ulp_start(uint32_t *func_addr) {
  ESP_ERROR_CHECK( ulp_run(func_addr - RTC_SLOW_MEM) );
}

void send_ulp_cmd(uint32_t cmd) {
    printf("Cmd: %d\n", cmd);
    ulp_cmd = cmd;
}

void dodo() {
        esp_sleep_enable_timer_wakeup(2000000);
        esp_light_sleep_start();

}

void app_main(void) {
    printf("Hello world!\n");

    rtc_gpio_init(2);
    rtc_gpio_set_direction(2, RTC_GPIO_MODE_OUTPUT_ONLY);
    rtc_gpio_set_level(2,0);
    ulp_init();
    ulp_start(&ulp_entry);
    while (true) {
        printf("Blink once\n");
        send_ulp_cmd(1);
        dodo();
        printf("Blink twice\n");
        send_ulp_cmd(2);
        dodo();
        printf("Fade in\n");
        send_ulp_cmd(3);
        dodo();
        printf("Fade out\n");
        send_ulp_cmd(4);
        dodo();
    }
}
