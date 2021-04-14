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
#include "esp32/ulp.h"
#include "ulp_main.h"
#include "driver/rtc_io.h"

extern const uint8_t bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t bin_end[]   asm("_binary_ulp_main_bin_end");
extern uint32_t ulp_cmd;

LED::LED(): fade_state(false) {
  rtc_gpio_init(GPIO_NUM_2);
  rtc_gpio_set_direction(GPIO_NUM_2, RTC_GPIO_MODE_OUTPUT_ONLY);
  ESP_ERROR_CHECK( ulp_load_binary(0, bin_start,(bin_end - bin_start) / sizeof(uint32_t)) );
  ulp_start(&ulp_entry);
}

void LED::ulp_start(uint32_t *func_addr) {
  ESP_ERROR_CHECK( ulp_run(func_addr - RTC_SLOW_MEM) );
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