// vim:et:sts=2:sw=2:si
#include "LED.h"
#include "esp32/ulp.h"
#include "ulp_main.h"

extern const uint8_t bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t bin_end[]   asm("_binary_ulp_main_bin_end");
extern uint32_t ulp_cmd;

LED::LED(): fade_state(false) {
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

void LED::toggleFade() {
  if (!fade_state) {
    fadeIn();
  } else {
    fadeOut();
  }
  fade_state = !fade_state;
}