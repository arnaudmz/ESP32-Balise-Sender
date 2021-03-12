// vim:et:sts=2:sw=2:si
#ifndef __Switches_h
#define __Switches_h
#include "driver/gpio.h"
#include "Config.h"

class Switches {
  public:
    Switches(Config *c);
    bool enabled();
    int getGroupMSBState();
    int getGroupLSBState();
    int getMassMSBState();
    int getMassLSBState();
  private:
    Config *config;
    int getIO(gpio_num_t g);
    void prepareIO(gpio_num_t g);
};
#endif //ifndef __Switches_h