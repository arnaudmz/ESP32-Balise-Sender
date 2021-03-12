
// vim:et:sts=2:sw=2:si
#include "Switches.h"

Switches::Switches(Config *config): config(config) {
  if (enabled()) {
    prepareIO(config->getGroupMSBPort());
    prepareIO(config->getGroupLSBPort());
    prepareIO(config->getMassMSBPort());
    prepareIO(config->getMassLSBPort());
  }
}

bool Switches::enabled() {
  return config->areSwitchesEnabled();
}

int Switches::getGroupMSBState() {
  return getIO(config->getGroupMSBPort());
}

int Switches::getGroupLSBState() {
  return getIO(config->getGroupLSBPort());
}

int Switches::getMassMSBState() {
  return getIO(config->getMassMSBPort());
}

int Switches::getMassLSBState() {
  return getIO(config->getMassLSBPort());
}

void Switches::prepareIO(gpio_num_t g) {
  gpio_pad_select_gpio(g);
  ESP_ERROR_CHECK( gpio_set_direction(g, GPIO_MODE_INPUT) );
}

int Switches::getIO(gpio_num_t g) {
  int r;
  gpio_pulldown_en(g);
  r = gpio_get_level(g);
  gpio_pulldown_dis(g);
  return r;
}