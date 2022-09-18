// vim:et:sts=2:sw=2:si
/*
ESP32-Balise-Sender, Model / Drone beacon
Copyright (C) 2021 Arnaud MAZIN
All rights reserved.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
#include "LED.h"
#ifdef CONFIG_IDF_TARGET_ESP32
#include "esp32/ulp.h"
#elif CONFIG_IDF_TARGET_ESP32S2
#include "esp32s2/ulp.h"
//#include "esp32s2/ulp_riscv.h"
// Waiting for the C++ compatible headers
extern "C" {
  esp_err_t ulp_riscv_run(void);
  esp_err_t ulp_riscv_load_binary(const uint8_t* program_binary, size_t program_size_bytes);
}
#endif // ifdef CONFIG_IDF_TARGET_ESP32
#if defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32)
#include "ulp_main.h"
extern const uint8_t bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t bin_end[]   asm("_binary_ulp_main_bin_end");
#endif // if defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32)

#if CONFIG_IDF_TARGET_ESP32C3
#include "driver/ledc.h"
#define BLINK_GPIO 3
#define MAX_DC 16383

ledc_timer_config_t ledc_timer;
ledc_channel_config_t ledc_channel;
#endif

#include "driver/rtc_io.h"

//#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
//#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#define LOG_LOCAL_LEVEL ESP_LOG_INFO
static constexpr char TAG[] = "Led";
#include "esp_log.h"

LED::LED(): fade_state(false) {
#if defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32)
  rtc_gpio_init(GPIO_NUM_2);
  rtc_gpio_set_direction(GPIO_NUM_2, RTC_GPIO_MODE_OUTPUT_ONLY);
#endif // if defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32)
#ifdef CONFIG_IDF_TARGET_ESP32
  ESP_ERROR_CHECK( ulp_load_binary(0, bin_start,(bin_end - bin_start) / sizeof(uint32_t)) );
  ESP_ERROR_CHECK( ulp_run(&ulp_entry - RTC_SLOW_MEM) );
#else
#ifdef CONFIG_IDF_TARGET_ESP32S2
  ESP_ERROR_CHECK( ulp_riscv_load_binary(bin_start, (bin_end - bin_start)) );
  ESP_ERROR_CHECK( ulp_riscv_run() );
#endif
#endif
#ifdef CONFIG_IDF_TARGET_ESP32C3
  ledc_timer.duty_resolution = LEDC_TIMER_14_BIT;
  ledc_timer.freq_hz         = 1000;                      // frequency of PWM signal
  ledc_timer.speed_mode      = LEDC_LOW_SPEED_MODE;    // timer mode
  ledc_timer.timer_num       = LEDC_TIMER_0;            // timer index
  ledc_timer.clk_cfg         = LEDC_AUTO_CLK;             // Auto select the source clock
  ledc_channel.channel       = LEDC_CHANNEL_0;
  ledc_channel.duty          = 0;
  ledc_channel.gpio_num      = BLINK_GPIO;
  ledc_channel.speed_mode    = LEDC_LOW_SPEED_MODE;
  ledc_channel.hpoint        = 0;
  ledc_channel.timer_sel     = LEDC_TIMER_0;
  ledc_channel.flags.output_invert = 1;
  ledc_timer_config(&ledc_timer);
  ledc_channel_config(&ledc_channel);
  ledc_fade_func_install(0);
#endif
}

void LED::blinkOnce() {
#if defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32)
  ulp_cmd = 1;
#endif
#ifdef CONFIG_IDF_TARGET_ESP32C3
  ledc_set_fade_time_and_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, MAX_DC, 1, LEDC_FADE_NO_WAIT);
  vTaskDelay(pdMS_TO_TICKS(100));
  ledc_set_fade_time_and_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0, 1, LEDC_FADE_NO_WAIT);
#endif
}

void LED::blinkTwice() {
#if defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32)
  ulp_cmd = 2;
#endif
#ifdef CONFIG_IDF_TARGET_ESP32C3
  ledc_set_fade_time_and_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, MAX_DC, 1, LEDC_FADE_NO_WAIT);
  vTaskDelay(pdMS_TO_TICKS(100));
  ledc_set_fade_time_and_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0, 1, LEDC_FADE_NO_WAIT);
  vTaskDelay(pdMS_TO_TICKS(100));
  ledc_set_fade_time_and_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, MAX_DC, 1, LEDC_FADE_NO_WAIT);
  vTaskDelay(pdMS_TO_TICKS(100));
  ledc_set_fade_time_and_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0, 1, LEDC_FADE_NO_WAIT);
#endif
}

void LED::fadeIn() {
#if defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32)
  ulp_cmd = 3;
#endif
#ifdef CONFIG_IDF_TARGET_ESP32C3
  ledc_set_fade_time_and_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, MAX_DC, 500, LEDC_FADE_NO_WAIT);
#endif
}

void LED::fadeOut() {
#if defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32)
  ulp_cmd = 4;
#endif
#ifdef CONFIG_IDF_TARGET_ESP32C3
  ledc_set_fade_time_and_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0, 500, LEDC_FADE_NO_WAIT);
#endif
}

void LED::blinkFastForever() {
#if defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32)
  ulp_cmd = 5;
#endif
#ifdef CONFIG_IDF_TARGET_ESP32C3
  ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, 5);
  ledc_set_fade_time_and_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 8191, 0, LEDC_FADE_NO_WAIT);
#endif
}

void LED::toggleFade() {
  if (!fade_state) {
    fadeIn();
  } else {
    fadeOut();
  }
  fade_state = !fade_state;
}
