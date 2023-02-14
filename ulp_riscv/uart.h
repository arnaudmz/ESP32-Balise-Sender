// vim:et:sts=2:sw=2:si
#include <stdint.h>

void uart_init();
void uart_prepare_for_tx();
void uart_prepare_for_rx();
bool uart_wait_for_start_bit(uint32_t timeout);
uint8_t uart_get_byte();
void uart_send_byte(uint8_t b);