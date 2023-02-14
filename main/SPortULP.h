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
#pragma once
#include "commons.h"
#include "Config.h"
#include "TinyGPS++.h"
#include "Beacon.h"
#include "Telemetry.h"

class SPortULP: public Telemetry {
  public:
    SPortULP(Config *c, TinyGPSPlus *gps, Beacon *beacon);
    virtual void handle(uint32_t end_ts);
  private:
    Config *config;
    TinyGPSPlus *gps;
    Beacon *beacon;
    uint32_t convertLatLon(float latLon, bool isLat);
    uint32_t convertDateTime(uint8_t yearOrHour, uint8_t monthOrMinute, uint8_t dayOrSecond, bool isDate);
};
