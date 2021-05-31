// vim:et:sts=2:sw=2:si
#include <stdint.h>
#include "ulp_riscv/ulp_riscv.h"
#include "ulp_riscv/ulp_riscv_utils.h"
#include "ulp_gpio.h"

/* this variable will be exported as a public symbol, visible from main CPU: */
volatile uint32_t cmd = 0;
#define LED_IO GPIO_NUM_21
#define LED_ON ulp_gpio_output_level(LED_IO, 0)
#define LED_OFF ulp_gpio_output_level(LED_IO, 1)
#define MS_TICKS 195

void delay(uint32_t duration) {
  for(volatile uint32_t i = 0; i < duration; i++) {
    ;
  }
}

void blink(uint32_t duration) {
  LED_ON;
  delay(duration);
  LED_OFF;
}

void fade_in() {
  for (uint8_t i = 0 ; i < 255; i++){
    LED_OFF;
    delay((255 - i) << 1);
    LED_ON;
    delay(i << 1);
  }
}

void fade_out() {
  for (uint8_t i = 0 ; i < 255; i++){
    LED_ON;
    delay((255 - i) << 1);
    LED_OFF;
    delay(i << 1);
  }
}

int main (void) {
  ulp_gpio_init(LED_IO);
  ulp_gpio_output_enable(LED_IO);
  while(1) {  
    switch(cmd) {
      case 1:
        blink(100 * MS_TICKS);
        cmd = 0;
        break;
      case 2:
        blink(100 * MS_TICKS);
        delay(100 * MS_TICKS);
        blink(100 * MS_TICKS);
        cmd = 0;
        break;
      case 3:
        fade_in();
        cmd = 0;
        break;
      case 4:
        fade_out();
        cmd = 0;
        break;
      case 5:
        blink(200 * MS_TICKS);
        delay(200 * MS_TICKS);
        break;
      default:
        break;
    }
  }
  return 0;
}
