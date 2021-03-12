
// vim:et:sts=2:sw=2:si
#ifndef __LED_h
#define __LED_h

#include "driver/uart.h"
#include "driver/i2c.h"
#include "TinyGPS++.h"
#define RX_BUF_SIZE 1024

class LED {
  public:
    LED();
    void begin();
    void blinkOnce();
    void blinkTwice();
    void fadeIn();
    void fadeOut();
    void toggleFade();
  private:
    void ulp_start(uint32_t *func_addr);
    bool fade_state;
};

#endif //ifndef __LED_h
