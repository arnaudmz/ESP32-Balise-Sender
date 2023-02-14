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
#include "ulp_main.h"
#include "ulp_shared_types.h"

//#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
//#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#define LOG_LOCAL_LEVEL ESP_LOG_INFO
static constexpr char TAG[] = "LED";
#include "esp_log.h"

LED::LED(): fade_state(false) {
}

void LED::blinkOnce() {
  ulp_led_cmd = LED_CMD_BLINK_ONCE;
  ESP_LOGI(TAG, "Blink Once");
}

void LED::blinkTwice() {
  ulp_led_cmd = LED_CMD_BLINK_TWICE;
}

void LED::fadeIn() {
  ulp_led_cmd = LED_CMD_FADE_IN;
}

void LED::fadeOut() {
  ulp_led_cmd = LED_CMD_FADE_OUT;
}

void LED::blinkFastForever() {
  ulp_led_cmd = LED_CMD_BLINK_FOREVER;
}

void LED::toggleFade() {
  if (!fade_state) {
    fadeIn();
  } else {
    fadeOut();
  }
  fade_state = !fade_state;
}
