
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
#ifndef __LED_h
#define __LED_h
#include "commons.h"

#include "driver/uart.h"
#include "driver/i2c.h"
#include "TinyGPS++.h"
#define RX_BUF_SIZE 1024

class LED {
  public:
    LED();
    void blinkOnce();
    void blinkTwice();
    void fadeIn();
    void fadeOut();
    void toggleFade();
    void blinkFastForever();
  private:
    bool fade_state;
};

#endif //ifndef __LED_h
