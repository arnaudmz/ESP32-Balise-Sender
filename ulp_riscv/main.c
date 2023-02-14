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
#include "uart.h"
#include "sport.h"
#include "led.h"

// should we start only LED loop or LED + FRSKY SPport loop ?
volatile uint32_t start_mode = START_MODE_LED_ONLY;

int main (void) {
  if (start_mode ==  START_MODE_LED_ONLY) {
    ;
  }
  uint8_t st[16];
  static uint32_t count = 0;
  dbg_init();
  led_init();
  uart_init();
  uint8_t old_val = 0;
  while(1) {
    if (uart_wait_for_start_bit(20)) {
      // got a start bit, read all bits + stop bit
      st[count++] = uart_get_byte();
    } else {
      led_handle();
      sport_handle(st, count);
      count = 0;
    }
  }
  return 0;
}
