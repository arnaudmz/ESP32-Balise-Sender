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

enum JetiReply {
  JETI_REPLY_ABSENT = -2,
  JETI_REPLY_INVALID = -1,
  JETI_REPLY_NONE = 0,
  JETI_REPLY_BUTTON_LEFT = 0x80,
  JETI_REPLY_BUTTON_DOWN = 0x40,
  JETI_REPLY_BUTTON_UP = 0x20,
  JETI_REPLY_BUTTON_RIGHT = 0x10
};

enum JetiMenuItem {
  JETI_MENU_HOME = 0,
  JETI_MENU_BEACON_ID,
  JETI_MENU_SAT_STATUS,
  JETI_MENU_SETTINGS_GROUP,
  JETI_MENU_SETTINGS_MASS,
  JETI_MENU_LAST
};

enum JetiMenuAction {
  JETI_MENU_ACTION_NONE = 0,
  JETI_MENU_ACTION_NOTIFY,
  JETI_MENU_ACTION_EXIT
};

class JetiScreen {
  public:
    JetiScreen(Config *config, TinyGPSPlus *gps, Beacon *beacon);
    void prepareScreen();
    JetiMenuAction navigate(JetiReply r);
    const char *getScreen() { return screen; };
  private:
    void setHomeScreen();
    void setBeaconIDScreen();
    void setSatStatusScreen();
    void setRollerMenuScreen(bool isGroup);
    uint8_t getNextIndex(int8_t index, bool direction, uint8_t max);
    bool saveNewPrefix();
    Config *config;
    TinyGPSPlus *gps;
    Beacon *beacon;
    char screen[33];
    JetiMenuItem currentMenuItem = JETI_MENU_HOME;
    uint8_t newGroup = 0;
    uint8_t newMass = 0;
    JetiReply lastReply = JETI_REPLY_NONE;
    uint32_t lastButtonChangeTS = 0;
};
