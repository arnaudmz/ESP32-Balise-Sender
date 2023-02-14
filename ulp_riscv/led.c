// vim:et:sts=2:sw=2:si

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
//#include "ulp_riscv.h"
#include "ulp_riscv_utils.h"
#include "ulp_riscv_gpio.h"

#include "ulp_shared_types.h"
#include "dbg.h"
#include "led.h"

#define LED_IO       GPIO_NUM_14

#define LED_ON       ulp_riscv_gpio_output_level(LED_IO, LOW)
#define LED_OFF      ulp_riscv_gpio_output_level(LED_IO, HIGH)

#define PAUSE_100MS 1400

// Shared varaibles with main CPU
volatile uint32_t led_cmd = 0;

void led_init() {
  ulp_riscv_gpio_init(LED_IO);
  ulp_riscv_gpio_output_enable(LED_IO);
}

void led_handle() {
  static uint32_t led_last_change_ts = 0;
  static uint8_t led_last_status = 0;
  switch (led_cmd) {
    case LED_CMD_BLINK_ONCE:
      if (led_last_status == 0) {
        // start LED
        LED_ON;
        led_last_status = 1;
        led_last_change_ts = 0;
      } else {
        // About 100ms
        if (led_last_change_ts >= PAUSE_100MS) {
          // stop LED
          LED_OFF;
          led_last_status = 0;
          led_cmd = LED_CMD_NONE;
        }
      }
      led_last_change_ts++;
      break;
    case LED_CMD_BLINK_TWICE:
      switch(led_last_status) {
        case 0:
          // start LED
          LED_ON;
          led_last_status = 1;
          led_last_change_ts = 0;
          break;
        case 1:
          // About 100ms
          if (led_last_change_ts >= PAUSE_100MS) {
            // stop LED
            LED_OFF;
            led_last_status = 2;
            led_last_change_ts = 0;
          }
          break;
        case 2:
          // About 100ms
          if (led_last_change_ts >= PAUSE_100MS) {
            // start LED
            LED_ON;
            led_last_status = 3;
            led_last_change_ts = 0;
          }
          break;
        case 3:
          // About 100ms
          if (led_last_change_ts >= PAUSE_100MS) {
            // stop LED
            LED_OFF;
            led_last_status = 0;
            led_cmd = LED_CMD_NONE;
          }
          break;
      }
      led_last_change_ts++;
      break;
    case LED_CMD_FADE_IN:
      if (led_last_status == 0) {
        LED_OFF;
        led_last_status = 127;
        led_last_change_ts = 0;
      } else if (led_last_status == 1) {
          LED_ON;
          led_last_status = 0;
          led_cmd = LED_CMD_NONE;
      } else if (led_last_change_ts == 128) {
        LED_OFF;
        led_last_change_ts = 0;
        led_last_status--;
      } else if (led_last_change_ts == led_last_status) {
        LED_ON;
      }
      led_last_change_ts++;
      break;
    case LED_CMD_FADE_OUT:
      if (led_last_status == 0) {
        LED_ON;
        led_last_status = 127;
        led_last_change_ts = 0;
      } else if (led_last_status == 1) {
          LED_OFF;
          led_last_status = 0;
          led_cmd = LED_CMD_NONE;
      } else if (led_last_change_ts == 128) {
        LED_ON;
        led_last_change_ts = 0;
        led_last_status--;
      } else if (led_last_change_ts == led_last_status) {
        LED_OFF;
      }
      led_last_change_ts++;
      break;
    case LED_CMD_BLINK_FOREVER:
      switch(led_last_status) {
        case 0:
          // start LED
          LED_ON;
          led_last_status = 1;
          led_last_change_ts = 0;
          break;
        case 1:
          // About 100ms
          if (led_last_change_ts >= PAUSE_100MS) {
            // stop LED
            LED_OFF;
            led_last_status = 2;
          led_last_change_ts = 0;
          }
          break;
        case 2:
          // About 100ms, go back to start
          if (led_last_change_ts >= PAUSE_100MS) {
            led_last_status = 0;
          }
          break;
      }
      led_last_change_ts++;
      break;
    case LED_CMD_NONE:
    default:
      // do nothing
      break;
  }
}
