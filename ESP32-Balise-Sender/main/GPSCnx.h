// vim:et:sts=2:sw=2:si
#ifndef __GPSCnx_h
#define __GPSCnx_h

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
    void computeMockMsg();
};

class GPSUARTCnx: public GPSCnx {
  public:
    GPSUARTCnx(Config *config, TinyGPSPlus *gps): GPSCnx(config, gps) {}
    virtual uint32_t waitForChars(int first_timeout_ms = 1000, int next_timeout_ms = 20, bool inject = true);
  protected:
    const uart_port_t uartPort = UART_NUM_2;
    void uartSendNMEA(const char *st);
    void uartWaitForSilence(int timeout_ms = 30);
};

class GPSBN220Cnx: public GPSUARTCnx {
  public:
    GPSBN220Cnx(Config *config, TinyGPSPlus *gps);
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
