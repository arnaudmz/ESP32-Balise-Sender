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
#ifndef __Beacon_h
#define __Beacon_h
#include "nvs.h"
#include "Config.h"
#include "LED.h"
#include "Switches.h"
#include "droneID_FR.h"
#include "TinyGPS++.h"

enum BeaconState {
  NO_GPS = 0,
  GPS_NOT_PRECISE,
  HAS_HOME,
  IN_FLIGHT
};

class Beacon {
  public:
    Beacon(Config *c, LED *l, Switches *s, droneIDFR *d, TinyGPSPlus *g);
    BeaconState getState();
    void handleData();
    bool hasSetHomeYet() { return hasSetHome; }
    bool hasTakenOffYet() { return hasTakenOff; }
    uint16_t getLastPrefix();
    void computeID();
    uint32_t getSentFramesCount() { return sentFrames; }

  private:
    void sendBeacon(const uint8_t *packet, const uint8_t to_send);
    void computeAndSendBeaconIfNeeded();
    char droneIDStr[33];
    Config *config;
    LED *led;
    Switches *switches;
    droneIDFR *droneID;
    TinyGPSPlus *gps;
    uint8_t headerSize;
    uint32_t sentFrames = 0;
    bool hasSetHome = false;
    double homeBestHDOP = 99.9;
    bool hasTakenOff = false;
    double homeAlt = 0.0;
    uint64_t beaconSec = 0;
    uint64_t gpsSec = 0;
    uint16_t lastPrefix = 0;
};
#endif //ifndef __Beacon_h
