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
#ifndef __Jeti_h
#define __Jeti_h
#include "driver/gpio.h"
#include "Config.h"
#include "TinyGPS++.h"
#include "Beacon.h"
#include "Telemetry.h"

enum JetiReply {
  JETI_REPLY_ABSENT = -2,
  JETI_REPLY_INVALID = -1,
  JETI_REPLY_NONE = 0,
  JETI_REPLY_BUTTON_LEFT = 0x80,
  JETI_REPLY_BUTTON_DOWN = 0x40,
  JETI_REPLY_BUTTON_UP = 0x20,
  JETI_REPLY_BUTTON_RIGHT = 0x10
};

enum MenuItem {
  JETI_MENU_HOME = 0,
  JETI_MENU_BEACON_ID,
  JETI_MENU_SAT_STATUS,
  JETI_MENU_SETTINGS_GROUP_MENU,
  JETI_MENU_SETTINGS_GROUP_NEW,
  JETI_MENU_SETTINGS_MASS_MENU,
  JETI_MENU_SETTINGS_MASS_NEW,
  JETI_MENU_LAST
};

class JetiTelemetry: public Telemetry {
  public:
    JetiTelemetry(Config *c, TinyGPSPlus *gps, Beacon *beacon);
    virtual void handle(uint32_t end_ts);
  private:
    uint32_t getCycleCount();
    uint8_t updateCRC(uint8_t c, uint8_t crc_seed);
    uint8_t writeAndUpdateCRC(uint8_t c, uint8_t crc_seed);
    void uartWrite(uint8_t byte, bool bit8);
    void switchToRX();
    void switchToTX();
    JetiReply lookForReply();
    void sendExAlarm(uint8_t byte);
#if 0
    void sendExMessage(const char *st);
    void sendExText(const char *st);
    void sendExMetricDesc(uint8_t metric_id, const char *value, const char *unit);
    void sendExMetricData(uint8_t metric_id);
#endif
    void sendText(const char *st);
    void prepareScreen();
    void setHomeScreen();
    void setBeaconIDScreen();
    void setSatStatusScreen();
    void setRollerMenuScreen(bool isGroup);
    uint8_t getNextIndex(int8_t index, bool direction, uint8_t max);
    void saveNewPrefix();
    void navigate(JetiReply r);
    Config *config;
    TinyGPSPlus *gps;
    Beacon *beacon;
    uint32_t bitTime;
    char screen[33];
    MenuItem currentMenuItem = JETI_MENU_HOME;
    JetiReply lastReply = JETI_REPLY_NONE;
    uint8_t newGroup = 0;
    uint8_t newMass = 0;
    bool notifyPrefChange = false;
    BeaconState lastNotifiedState = NO_GPS;
};
#endif //ifndef __Jeti_h
