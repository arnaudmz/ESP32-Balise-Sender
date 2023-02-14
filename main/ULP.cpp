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
#include "ULP.h"
#ifdef CONFIG_IDF_TARGET_ESP32
#include "esp32/ulp.h"
#elif CONFIG_IDF_TARGET_ESP32S3
#include "ulp_riscv.h"
#endif
#include "ulp_main.h"
#include "ulp_shared_types.h"

extern const uint8_t bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t bin_end[]   asm("_binary_ulp_main_bin_end");

static constexpr char TAG[] = "ULP";
#include "esp_log.h"

ULP::ULP(Config *config, bool isMaintenance) {
#ifdef CONFIG_IDF_TARGET_ESP32
  rtc_gpio_init(GPIO_NUM_2);
  rtc_gpio_set_direction(GPIO_NUM_2, RTC_GPIO_MODE_OUTPUT_ONLY);
  ESP_ERROR_CHECK( ulp_load_binary(0, bin_start,(bin_end - bin_start) / sizeof(uint32_t)) );
  ESP_ERROR_CHECK( ulp_run(&ulp_entry - RTC_SLOW_MEM) );
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
  ulp_start_mode = (config->getTelemetryMode() == TELEMETRY_FRSP && !isMaintenance)? START_MODE_LED_AND_SPORT: START_MODE_LED_ONLY;
  ESP_LOGI(TAG, "Starting in mode %ld", ulp_start_mode);
  ESP_ERROR_CHECK( ulp_riscv_load_binary(bin_start, (bin_end - bin_start)) );
  ESP_ERROR_CHECK( ulp_riscv_run() );
#endif
}
