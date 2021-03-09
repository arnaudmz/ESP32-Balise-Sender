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
    //ulp_start(&ulp_entry);
}

void dodo() {
        //fflush(stdout);
        //uart_wait_tx_done(CONFIG_ESP_CONSOLE_UART_NUM, pdMS_TO_TICKS(1000));
        //uart_wait_tx_done(UART_NUM_0, pdMS_TO_TICKS(100));
        //vTaskDelay(pdMS_TO_TICKS(5000));
        //esp_sleep_enable_gpio_wakeup();
        esp_sleep_enable_timer_wakeup(2000000);
        //ESP_ERROR_CHECK( esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON) );
        //ESP_ERROR_CHECK( esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_ON) );
        esp_light_sleep_start();
        //vTaskDelay(2000 / portTICK_PERIOD_MS);
        //vTaskDelay(pdMS_TO_TICKS(20000));

}

void app_main(void) {
    printf("Hello world!\n");

//    rtc_gpio_init(2);
//    rtc_gpio_set_direction(2, RTC_GPIO_MODE_OUTPUT_ONLY);
//    rtc_gpio_set_level(2, 1);
    ulp_init();
    ulp_start(&ulp_entry);
    //send_ulp_cmd(3);

    //for (int i = 10; i >= 0; i--) {
    //    printf("Restarting in %d seconds...\n", i);
    //    vTaskDelay(1000 / portTICK_PERIOD_MS);
    //}
    //printf("Restarting now.\n");
    //fflush(stdout);
    //esp_restart();
    //int dir = 1;
    while (true) {
        send_ulp_cmd(1);
        printf("Blink once\n");
        ////ulp_start(&ulp_blink_once);
        dodo();
        printf("Blink twice\n");
        send_ulp_cmd(2);
        ////ulp_start(&ulp_blink_twice);
        dodo();
        printf("Fade in\n");
        send_ulp_cmd(3);
        //ulp_start(&ulp_fade_in);
        dodo();
        printf("Fade out\n");
        send_ulp_cmd(4);
        //ulp_start(&ulp_fade_out);
        dodo();
    }
}
