// vim:et:sts=2:sw=2:si
#include "ulp_riscv_register_ops.h"
#include "ulp_riscv_utils.h"
#include "ulp_riscv_gpio.h"

#include "ulp_shared_types.h"

#define DBG_IO       GPIO_NUM_18
#define DBG_ON       ulp_riscv_gpio_output_level(DBG_IO, HIGH)
#define DBG_OFF      ulp_riscv_gpio_output_level(DBG_IO, LOW)

void dbg_init();
