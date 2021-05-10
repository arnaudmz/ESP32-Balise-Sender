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
#include "JetiScreen.h"

enum JetiMetricKind {
  JETI_METRIC_KIND_SENSOR_DESC = 0,
  JETI_METRIC_KIND_PREFIX,
  JETI_METRIC_KIND_SPEED,
  JETI_METRIC_KIND_ALT,
  JETI_METRIC_KIND_HDOP,
  JETI_METRIC_KIND_SAT,
  JETI_METRIC_KIND_STAT,
  JETI_METRIC_KIND_LAST
};

enum JetiMetricType {
  JETI_METRIC_TYPE_SENSOR_DESC = -1,
  JETI_METRIC_TYPE_INT6 = 0,          // 0
  JETI_METRIC_TYPE_INT14 = 1         // 1
//  JETI_METRIC_TYPE_INT14_RESERVED_2,  // 2 unused
//  JETI_METRIC_TYPE_INT14_RESERVED_3,  // 3
//  JETI_METRIC_TYPE_INT22,             // 4
//  JETI_METRIC_TYPE_INT22_DATE_TIME,   // 5
//  JETI_METRIC_TYPE_INT22_RESERVED_6,  // 6
//  JETI_METRIC_TYPE_INT22_RESERVED_7,  // 7
//  JETI_METRIC_TYPE_INT30,             // 8
//  JETI_METRIC_TYPE_INT30_GPS,         // 9
//  JETI_METRIC_TYPE_INT30_RESERVED_10, // 10
//  JETI_METRIC_TYPE_INT30_RESERVED_11, // 11
//  JETI_METRIC_TYPE_INT38_RESERVED_12, // 12
//  JETI_METRIC_TYPE_INT38_RESERVED_13, // 13
//  JETI_METRIC_TYPE_INT38_RESERVED_14, // 14
//  JETI_METRIC_TYPE_INT38_RESERVED_15  // 15
};

enum FrameRequestResult {
  FRAME_ABSENT = 0,
  FRAME_INVALID,
  FRAME_PRESENT,
};

#define JETI_DEVICE_MANUFACTOR_MSB 0xa4
#define JETI_DEVICE_MANUFACTOR_LSB 0x11
#define JETI_DEVICE_DEVICE_MSB     0xca
#define JETI_DEVICE_DEVICE_LSB     0xfe

class JetiMetric {
  public:
    JetiMetric(JetiMetricKind kind, JetiMetricType type, const char *description, const char *unit):
      kind(kind), type(type), description(description), unit(unit) {}
    uint8_t writeDesc(uint8_t *buffer, uint8_t pos); // append description to payload
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
  protected:
    JetiTelemetry(Config *c, TinyGPSPlus *gps, Beacon *beacon, JetiScreen screen): config(c), gps(gps), beacon(beacon), screen(screen) {};
    uint32_t getCycleCount();
    uint8_t updateCRC(uint8_t c, uint8_t crc_seed);
    Config *config;
    TinyGPSPlus *gps;
    Beacon *beacon;
    uint8_t sensor_id = JETI_METRIC_KIND_SENSOR_DESC;
    bool send_desc = true;
    JetiScreen screen;
  private:
    void uartWrite(uint8_t byte, bool bit8);
    void switchToRX();
    void switchToTX();
    JetiReply lookForReply();
#if 0
    void sendExMessage(const char *st);
    void sendExAlarm(uint8_t byte);
#endif
    void sendExMetricData();
    void sendExMetricDesc();
    void sendJetiBoxScreen(const char *st);
    uint8_t exFrameBuffer[26] = {
        0x00, // will be computed for frame type + length
        JETI_DEVICE_MANUFACTOR_LSB, // Manufactor ID LSB
        JETI_DEVICE_MANUFACTOR_MSB, // Manufactor ID MSB
        JETI_DEVICE_DEVICE_LSB,     // Device ID LSB
        JETI_DEVICE_DEVICE_MSB,     // Device ID MSB
        0x00  // Always 0
    };
    uint32_t bitTime;
};

class JetiExBusTelemetry: public JetiTelemetry {
  public:
    JetiExBusTelemetry(Config *c, TinyGPSPlus *gps, Beacon *beacon);
    virtual void handle(uint32_t end_ts);
  private:
    bool sendNextTelemetryFrame(uint8_t id);
    void sendScreen(uint8_t id);
    uint32_t readChars();
    uint16_t updateCRC16_CCITT(uint16_t crc, uint8_t data);
    bool validateCRC16_CCITT(const uint8_t *buffer, uint8_t len);
    void sendResponse(const uint8_t *buffer, uint8_t len);
    FrameRequestResult isLastFrameValid(const uint8_t *buffer, uint8_t nb_chars, uint8_t len, uint8_t request);
    const uart_port_t uartPort = UART_NUM_1;
    uint8_t rxBuffer[RX_BUF_SIZE];
    uint8_t exBusFrameTelemetryBuffer[40] = {
        0x3B, // Answer header
        0x01, // Answer header
        0x00, // length (will be computed later on)
        0x00, // ID will be copied from request
        0x3a, // Type (0x3a for telemetry)
        0x00, // sub-length (will be computed later on)
        0x9f, // Teletry (anything that ends in 0xf)
        0x00, // will be computed for frame type + length
        JETI_DEVICE_MANUFACTOR_LSB, // Manufactor ID LSB
        JETI_DEVICE_MANUFACTOR_MSB, // Manufactor ID MSB
        JETI_DEVICE_DEVICE_LSB,     // Device ID LSB
        JETI_DEVICE_DEVICE_MSB,     // Device ID MSB
        0x00  // Always 0
    };
    uint8_t exBusFrameJetiBoxMenuBuffer[40] = {
        0x3B, // Answer header
        0x01, // Answer header
        0x28, // length (always 40 for Screen)
        0x00, // ID will be copied from request
        0x3b, // Type (0x3b for JetiScreen)
        0x20  // sub-length (always 32 for Screen)
    };
};
#endif //ifndef __Jeti_h
