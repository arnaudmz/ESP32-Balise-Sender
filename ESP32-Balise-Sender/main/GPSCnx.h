// vim:et:sts=2:sw=2:si
#ifndef __GPSCnx_h
#define __GPSCnx_h

#include "driver/uart.h"
#include "driver/i2c.h"
#include "TinyGPS++.h"
#include "Beacon.h"
#define RX_BUF_SIZE 1024

class GPSCnx {
  public:
    GPSCnx(TinyGPSPlus *gps): gps(gps) {}
    void begin();
    uint32_t waitForChars(int first_timeout_ms = 1000, int next_timeout_ms = 20, bool inject = true);
#ifdef CONFIG_BEACON_GPS_MOCK
    void setBeacon(Beacon *b);
#endif //#ifdef CONFIG_BEACON_GPS_MOCK
  private:
    TinyGPSPlus *gps;
    uint8_t rxBuffer[RX_BUF_SIZE];
#ifdef CONFIG_BEACON_GPS_MOCK
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
#endif //#ifdef CONFIG_BEACON_GPS_MOCK
#ifdef CONFIG_BEACON_GPS_L96_I2C
    const i2c_port_t i2cPort  = I2C_NUM_0;
    uint8_t i2cBuf[I2C_BUF_SIZE];
    void i2cSetup();
#endif //ifdef CONFIG_BEACON_GPS_L96_I2C
#if defined(CONFIG_BEACON_GPS_L96_UART) || defined(CONFIG_BEACON_GPS_L80R_UART) || defined(CONFIG_BEACON_GPS_BN_220_UART)
    const uart_port_t uartPort = UART_NUM_2;
    void uartSetup();
    void uartSendPMTK(const char *st);
    void uartWaitForSilence(int timeout_ms = 30);
#endif //#if defined(CONFIG_BEACON_GPS_L96_UART) || defined(CONFIG_BEACON_GPS_L80R_UART) || defined(CONFIG_BEACON_GPS_BN_220_UART)
    uint8_t computePMTKCksum(const char *st);
};

#endif //ifndef __GPSCnx_h
