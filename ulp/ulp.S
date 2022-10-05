#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/soc_ulp.h"
#define GPIO_2_RTC_NUM 12
#define LED_ON WRITE_RTC_REG(RTC_GPIO_OUT_REG, RTC_GPIO_OUT_DATA_S + GPIO_2_RTC_NUM, 1, 0)
#define LED_OFF WRITE_RTC_REG(RTC_GPIO_OUT_REG, RTC_GPIO_OUT_DATA_S + GPIO_2_RTC_NUM , 1, 1)

  .bss                        
  
  .global cmd
cmd:
  .long 0

.text

// Entry Points ================================================================
  .global entry
entry:
  LED_ON
main_loop:
  move r3, cmd
  ld r0, r3, 0
  jumpr blink_fast_forever_logic, 5, ge
  jumpr fade_out_logic, 4, ge
  jumpr fade_in_logic, 3, ge
  jumpr blink_twice_logic, 2, ge
  jumpr blink_once_logic, 1, ge
  wait 65535
  jump main_loop
main_loop_end:
  move r3, cmd
  move r0, 0
  st r0, r3, 0
  jump main_loop

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
