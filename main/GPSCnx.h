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
#ifndef __GPSCnx_h
#define __GPSCnx_h
#include "commons.h"

#include "driver/uart.h"
#include "driver/i2c.h"
#include "TinyGPS++.h"
#include "Config.h"
#include "Beacon.h"
#define RX_BUF_SIZE 1024

class GPSCnx {
  public:
    GPSCnx(Config *config, TinyGPSPlus *gps): config(config), gps(gps) {}
    virtual uint32_t waitForChars(int first_timeout_ms = 1000, int next_timeout_ms = 20, bool inject = true);
    virtual void lowPower(uint32_t delay_ms = 0);
  protected:
    uint8_t computeNMEACksum(const char *st);
    void injectIfNeeded(uint32_t nb_chars, bool inject);
    Config *config;
    TinyGPSPlus *gps;
    uint8_t rxBuffer[RX_BUF_SIZE];
};

class GPSMockCnx: public GPSCnx {
  public:
    GPSMockCnx(Config *config, TinyGPSPlus *gps, Beacon *b): GPSCnx(config, gps), beacon(b) { }
    virtual uint32_t waitForChars(int first_timeout_ms = 1000, int next_timeout_ms = 20, bool inject = true);
  private:
    Beacon *beacon;
    char mock_msg[256];
    double mockGPSHomeLat = 4843.20138;
    double mockGPSHomeLong = 211.47344;
    double mockGPSHomeTS = 145203;
    double mockGPSRaduis = 0.30;
    int mockGPSHomeAlt = 148;
    int mockGPSMaxHeight = 150;
    int mockGPSMinHeight = 10;
    double hdop = 6.0;
    uint8_t nb_sat = 1;
    int alt;
    double speed_in_knots = 0.0;
    void computeMockMsg();
};

class GPSUARTCnx: public GPSCnx {
  public:
    GPSUARTCnx(Config *config, TinyGPSPlus *gps): GPSCnx(config, gps) {}
    virtual uint32_t waitForChars(int first_timeout_ms = 1000, int next_timeout_ms = 20, bool inject = true);
  protected:
#ifdef CONFIG_IDF_TARGET_ESP32
    const uart_port_t uartPort = UART_NUM_2;
#else
    const uart_port_t uartPort = UART_NUM_0;
#endif
    void uartSendNMEA(const char *st);
    void uartWaitForSilence(int timeout_ms = 30);
};

class GPSBN220Cnx: public GPSUARTCnx {
  public:
    GPSBN220Cnx(Config *config, TinyGPSPlus *gps);
};

class GPSAT6558Cnx: public GPSUARTCnx {
  public:
    GPSAT6558Cnx(Config *config, TinyGPSPlus *gps);
};

class GPSAT6558ULPCnx: public GPSAT6558Cnx {
  public:
    GPSAT6558ULPCnx(Config *config, TinyGPSPlus *gps);
    virtual void lowPower(uint32_t delay_ms = 0);
    virtual uint32_t waitForChars(int first_timeout_ms = 1000, int next_timeout_ms = 20, bool inject = true);
};

class GPSPPSUARTCnx: public GPSUARTCnx {
  public:
    GPSPPSUARTCnx(Config *config, TinyGPSPlus *gps);
    virtual void lowPower(uint32_t delay_ms = 0);
};

class GPSL80RCnx: public GPSPPSUARTCnx {
  public:
    GPSL80RCnx(Config *config, TinyGPSPlus *gps);
};

class GPSL96Cnx: public GPSPPSUARTCnx {
  public:
    GPSL96Cnx(Config *config, TinyGPSPlus *gps);
};

#endif //ifndef __GPSCnx_h
