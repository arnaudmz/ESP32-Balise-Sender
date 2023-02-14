// vim:et:sts=2:sw=2:si

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
//#include "ulp_riscv_register_ops.h"
#include "ulp_riscv_utils.h"
#include "ulp_riscv_gpio.h"

#include "ulp_shared_types.h"
#include "dbg.h"
#include "uart.h"

#define UART_IO     GPIO_NUM_8

#define UART_LEVEL  ulp_riscv_gpio_get_level(UART_IO)
#define UART_OUT(l) ulp_riscv_gpio_output_level(UART_IO, l)

#define BAUDRATE 57600
#define BIT_DURATION_CYCLES ( ULP_RISCV_CYCLES_PER_US * ((1000*1000) / BAUDRATE) )
//#define SEND_DELAY ulp_riscv_delay_cycles(BIT_DURATION_CYCLES - 121)
#define SEND_DELAY ulp_riscv_delay_cycles(BIT_DURATION_CYCLES - 100)

void uart_init() {
  ulp_riscv_gpio_init(UART_IO);
  uart_prepare_for_rx();
}

void uart_prepare_for_tx() {
  ulp_riscv_gpio_input_disable(UART_IO);
  ulp_riscv_gpio_output_enable(UART_IO);
}

void uart_prepare_for_rx() {
  ulp_riscv_gpio_output_disable(UART_IO);
  ulp_riscv_gpio_input_enable(UART_IO);
}

// true == got a start bit, false == timeout
bool uart_wait_for_start_bit(uint32_t timeout) {
  for(uint32_t i = 0; i < timeout; i++) {
    if (UART_LEVEL) {
      return true;
    }
  }
  return false;
}

uint8_t uart_get_byte() {
  uint8_t v = 0;
  ulp_riscv_delay_cycles(BIT_DURATION_CYCLES - 20);
  for (uint8_t i = 0; i < 8; i++) {
    DBG_ON;
    v |= (!UART_LEVEL << i);
    DBG_OFF;
    ulp_riscv_delay_cycles(BIT_DURATION_CYCLES - 220);
  }
  DBG_ON;
  DBG_OFF;
  return v;
}

void uart_send_byte(uint8_t b) {
  // start bit
  UART_OUT(HIGH);
  SEND_DELAY;
  UART_OUT(b & (1 << 0)? LOW: HIGH);
  SEND_DELAY;
  UART_OUT(b & (1 << 1)? LOW: HIGH);
  SEND_DELAY;
  UART_OUT(b & (1 << 2)? LOW: HIGH);
  SEND_DELAY;
  UART_OUT(b & (1 << 3)? LOW: HIGH);
  SEND_DELAY;
  UART_OUT(b & (1 << 4)? LOW: HIGH);
  SEND_DELAY;
  UART_OUT(b & (1 << 5)? LOW: HIGH);
  SEND_DELAY;
  UART_OUT(b & (1 << 6)? LOW: HIGH);
  SEND_DELAY;
  UART_OUT(b & (1 << 7)? LOW: HIGH);
  SEND_DELAY;
  // stop bit
  UART_OUT(LOW);
  SEND_DELAY;
}
