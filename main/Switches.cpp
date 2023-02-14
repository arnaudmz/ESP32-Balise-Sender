
// vim:et:sts=2:sw=2:si
/*
ESP32-Balise-Sender, Model / Drone beacon
Copyright (C) 2021 Arnaud MAZIN
All rights reserved.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
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
  gpio_reset_pin(g);
  ESP_ERROR_CHECK( gpio_set_direction(g, GPIO_MODE_INPUT) );
}

int Switches::getIO(gpio_num_t g) {
  int r;
  gpio_pulldown_en(g);
  r = gpio_get_level(g);
  gpio_pulldown_dis(g);
  return r;
}
