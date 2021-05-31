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
#else
#include "esp32s2/ulp.h"
//#include "esp32s2/ulp_riscv.h"
// Waiting for the C++ compatible headers
extern "C" {
  esp_err_t ulp_riscv_run(void);
  esp_err_t ulp_riscv_load_binary(const uint8_t* program_binary, size_t program_size_bytes);
}
#endif
#include "ulp_main.h"
#include "driver/rtc_io.h"

//#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
//#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#define LOG_LOCAL_LEVEL ESP_LOG_INFO
static constexpr char TAG[] = "Led";
#include "esp_log.h"

extern const uint8_t bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t bin_end[]   asm("_binary_ulp_main_bin_end");

LED::LED(): fade_state(false) {
  rtc_gpio_init(GPIO_NUM_2);
  rtc_gpio_set_direction(GPIO_NUM_2, RTC_GPIO_MODE_OUTPUT_ONLY);
#ifdef CONFIG_IDF_TARGET_ESP32
  ESP_ERROR_CHECK( ulp_load_binary(0, bin_start,(bin_end - bin_start) / sizeof(uint32_t)) );
  ESP_ERROR_CHECK( ulp_run(&ulp_entry - RTC_SLOW_MEM) );
#else
  ESP_ERROR_CHECK( ulp_riscv_load_binary(bin_start, (bin_end - bin_start)) );
  ESP_ERROR_CHECK( ulp_riscv_run() );
#endif
}

void LED::blinkOnce() {
  ulp_cmd = 1;
}

void LED::blinkTwice() {
  ulp_cmd = 2;
}

void LED::fadeIn() {
  ulp_cmd = 3;
}

void LED::fadeOut() {
  ulp_cmd = 4;
}

void LED::blinkFastForever() {
  ulp_cmd = 5;
}

void LED::toggleFade() {
  if (!fade_state) {
    fadeIn();
  } else {
    fadeOut();
  }
  fade_state = !fade_state;
}
