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

enum JetiMetricKind {
  JETI_METRIC_KIND_SENSOR_DESC = 0,
  JETI_METRIC_KIND_PREFIX,
  JETI_METRIC_KIND_SPEED,
  JETI_METRIC_KIND_ALT,
  JETI_METRIC_KIND_HDOP,
  JETI_METRIC_KIND_SAT,
  JETI_METRIC_KIND_LAST
};

enum JetiMetricType {
  JETI_METRIC_TYPE_SENSOR_DESC = -1,
  JETI_METRIC_TYPE_INT6 = 0,          // 0
  JETI_METRIC_TYPE_INT14,             // 1
  JETI_METRIC_TYPE_INT14_RESERVED_2,  // 2
  JETI_METRIC_TYPE_INT14_RESERVED_3,  // 3
  JETI_METRIC_TYPE_INT22,             // 4
  JETI_METRIC_TYPE_INT22_DATE_TIME,   // 5
  JETI_METRIC_TYPE_INT22_RESERVED_6,  // 6
  JETI_METRIC_TYPE_INT22_RESERVED_7,  // 7
  JETI_METRIC_TYPE_INT30,             // 8
  JETI_METRIC_TYPE_INT30_GPS,         // 9
  JETI_METRIC_TYPE_INT30_RESERVED_10, // 10
  JETI_METRIC_TYPE_INT30_RESERVED_11, // 11
  JETI_METRIC_TYPE_INT38_RESERVED_12, // 12
  JETI_METRIC_TYPE_INT38_RESERVED_13, // 13
  JETI_METRIC_TYPE_INT38_RESERVED_14, // 14
  JETI_METRIC_TYPE_INT38_RESERVED_15  // 15
};

class JetiMetric {
  public:
    JetiMetric(JetiMetricKind kind, JetiMetricType type, const char *description, const char *unit):
      kind(kind), type(type), description(description), unit(unit) {}
    uint8_t writeDesc(uint8_t *buffer, uint8_t pos); // append description to payload
#if 0
    uint8_t getDataLength(); // compute length of data (to see if it fits in a data frame)
#endif
    uint8_t writeData(uint8_t *buffer, uint8_t pos, TinyGPSPlus *gps, Beacon *beacon); // append this metric data to payload
  private:
    uint8_t formatINT14b(uint8_t *buffer, uint8_t pos, int16_t value, uint8_t decimals);
    uint8_t formatINT6b(uint8_t *buffer, uint8_t pos, int8_t value, uint8_t decimals);
    JetiMetricKind kind;
    JetiMetricType type;
    const char *description;
    const char *unit;
};

class JetiTelemetry: public Telemetry {
  public:
    JetiTelemetry(Config *c, TinyGPSPlus *gps, Beacon *beacon);
    virtual void handle(uint32_t end_ts);
  private:
    uint32_t getCycleCount();
    uint8_t updateCRC(uint8_t c, uint8_t crc_seed);
    void uartWrite(uint8_t byte, bool bit8);
    void switchToRX();
    void switchToTX();
    JetiReply lookForReply();
    void sendExAlarm(uint8_t byte);
#if 0
    uint8_t writeAndUpdateCRC(uint8_t c, uint8_t crc_seed);
    void sendExMessage(const char *st);
#endif
    void sendExMetricData();
    void sendExMetricDesc();
    void sendText(const char *st);
    void prepareScreen();
    void setHomeScreen();
    void setBeaconIDScreen();
    void setSatStatusScreen();
    void setRollerMenuScreen(bool isGroup);
    uint8_t getNextIndex(int8_t index, bool direction, uint8_t max);
    void saveNewPrefix();
    void navigate(JetiReply r);
    uint8_t exFrameBuffer[26] = {
        0x00, // will be computed for frame type + length
        0x11, // manufactor ID MSB
        0xA4, // manufactor ID LSB
        0x11, // Device ID MSB
        0xA4, // Device ID LSB
        0x00  // Always 0
    };
    Config *config;
    TinyGPSPlus *gps;
    Beacon *beacon;
    uint32_t bitTime;
    char screen[33];
    MenuItem currentMenuItem = JETI_MENU_HOME;
    //JetiReply lastReply = JETI_REPLY_NONE;
    uint8_t newGroup = 0;
    uint8_t newMass = 0;
    bool notifyPrefChange = false;
    BeaconState lastNotifiedState = NO_GPS;
    uint8_t sensor_id = JETI_METRIC_KIND_SENSOR_DESC;
    bool send_desc = true;
};
#endif //ifndef __Jeti_h
