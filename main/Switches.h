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