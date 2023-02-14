// vim:et:sts=2:sw=2:si

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "dbg.h"


void dbg_init() {
  ulp_riscv_gpio_init(DBG_IO);
  ulp_riscv_gpio_output_enable(DBG_IO);
}
