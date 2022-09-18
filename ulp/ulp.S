#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/soc_ulp.h"
#define GPIO_2_RTC_NUM 12
#define GPIO_33_RTC_NUM 8
#define LED_ON WRITE_RTC_REG(RTC_GPIO_OUT_REG, RTC_GPIO_OUT_DATA_S + GPIO_2_RTC_NUM, 1, 0)
#define LED_OFF WRITE_RTC_REG(RTC_GPIO_OUT_REG, RTC_GPIO_OUT_DATA_S + GPIO_2_RTC_NUM , 1, 1)
#define RX_BUF_MAX 200
#define GET_RX_BIT READ_RTC_REG(RTC_GPIO_IN_REG, RTC_GPIO_IN_NEXT_S + GPIO_33_RTC_NUM, 1)

  .bss                        
  
  .global cmd
cmd:
  .long 0

  .global rx_buf_len
rx_buf_len:
  .long 0
  
  .global rx_buf
rx_buf:
  .fill RX_BUF_MAX * 4

.text

// Entry Points ================================================================
  .global entry
entry:
  //LED_OFF
wait_for_start_bit_forever:
  GET_RX_BIT
  jumpr wait_for_start_bit_forever, 1, eq
  move r0, 0
  move r2, rx_buf_len
  st r0, r2, 0 // clear rx_buf by setting rx_buf_len to 0
  jump got_start_bit
wait_for_start_bit:
  move r1, 50
wait_again_for_start_bit:
  GET_RX_BIT
  jumpr got_start_bit, 0, eq
  sub r1, r1, 1
  jump timeout, eq
  jump wait_again_for_start_bit
got_start_bit:
  wait 16 
  stage_rst
  move r1, 0
next_bit_get:
  // get in the middle of next bit
  wait 28
  //wait 6
  //LED_ON
  GET_RX_BIT
  wait 4
  //LED_OFF
  // shift + store r0
  rsh r1, r1, 1
  lsh r0, r0, 7
  or r1, r1, r0
  stage_inc 1
  jumps next_bit_get, 8, lt // loop 8 times
  // I have my 8 bits in LSB of R1
  // Do I have enough room in buffer?
  move r0, RX_BUF_MAX
  move r2, rx_buf_len
  ld r2, r2, 0
  sub r0, r0, r2 // r0 = RX_BUF_MAX - rx_buf_len
  jump wait_for_start_bit, eq // buffer is full, don't store byte
  move r0, rx_buf
  add r0, r0, r2
  st r1, r0, 0
  add r2, r2, 1
  move r0, rx_buf_len
  st r2, r0, 0 // rx_buf_len ++
  jump wait_for_start_bit

timeout:
wake_up:
  /* Check if the system can be woken up */
  READ_RTC_FIELD(RTC_CNTL_LOW_POWER_ST_REG, RTC_CNTL_RDY_FOR_WAKEUP)
  and r0, r0, 1
  jump wake_up, eq
  wake
  // Next, do some LED animation before reading next UART packet
  // supposed to have plenty of time. GPS talk every second
  move r3, cmd
  ld r0, r3, 0
  jumpr blink_fast_forever_logic, 5, ge
  jumpr fade_out_logic, 4, ge
  jumpr fade_in_logic, 3, ge
  jumpr blink_twice_logic, 2, ge
  jumpr blink_once_logic, 1, ge
  jump wait_for_start_bit_forever
main_loop_end:
  move r3, cmd
  move r0, 0
  st r0, r3, 0
  jump wait_for_start_bit_forever

led_off_and_back_to_main_loop:
  LED_OFF
  jump main_loop_end
  
delay:
  wait  80                // wait 80 clock ticks at 8MHz -> 0.01ms
  sub   r1, r1, 1         // decrement ms count
  jump  r2, eq            // if ms count is zero then return to caller
  jump  delay             // else continue to wait

// Logic stuff =================================================================

// ======================================BLINK ONCE=============================
blink_once_logic:
  LED_ON
  move r1, 10000
  move r2, led_off_and_back_to_main_loop
  jump delay

// ======================================BLINK TWICE=============================
blink_twice_logic:
  LED_ON
  move r1, 10000
  move r2, blink_twice_logic_off_1
  jump delay
blink_twice_logic_off_1:
  LED_OFF
  move r1, 10000
  move r2, blink_twice_logic_on_2
  jump delay
blink_twice_logic_on_2:
  LED_ON
  move r1, 10000
  move r2, led_off_and_back_to_main_loop
  jump delay

.set fade_loops, 200
// =====================================FADE IN=================================
fade_in_logic:
  move r3, fade_loops
fade_in_loop_off:
  LED_OFF
  move r1, r3
  move r2, fade_in_loop_on
  jump delay
fade_in_loop_on:
  LED_ON
  move r1, (fade_loops + 1)
  sub r1, r1, r3
  move r2, fade_in_loop_next
  jump delay
fade_in_loop_next:
  sub r3, r3, 1
  jump main_loop_end, eq
  jump fade_in_loop_off

// =====================================FADE OUT=================================
fade_out_logic:
  move r3, fade_loops
  //move r0, fade_out_loop_next
fade_out_loop_on:
  LED_ON
  move r1, r3
  move r2, fade_out_loop_off
  jump delay
fade_out_loop_off:
  LED_OFF
  move r1, (fade_loops + 1)
  sub r1, r1, r3
  move r2, fade_out_loop_next
  jump delay
fade_out_loop_next:
  sub r3, r3, 1
  jump main_loop_end, eq
  jump fade_out_loop_on

// ======================================BLINK FAST FOREVER=====================
blink_fast_forever_logic:
  LED_ON
  move r1, 10000
  move r2, blink_fast_forever_off
  jump delay
blink_fast_forever_off:
  LED_OFF
  move r1, 10000
  move r2, blink_fast_forever_logic
  jump delay  
