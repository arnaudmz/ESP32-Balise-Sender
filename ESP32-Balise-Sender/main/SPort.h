
// vim:et:sts=2:sw=2:si
#ifndef __SPort_h
#define __SPort_h
#include "driver/gpio.h"
#include "driver/uart.h"
#include "Config.h"
#include "TinyGPS++.h"
#include "Beacon.h"

#define RX_BUF_SIZE 1024

enum SPortMetricType {
//  SPORT_METRIC_LAT = 0,
//  SPORT_METRIC_LON, 
  SPORT_METRIC_ALT = 0,
  SPORT_METRIC_SPEED,
//  SPORT_METRIC_COG,
//  SPORT_METRIC_DATE,
//  SPORT_METRIC_TIME,
  SPORT_METRIC_STAT,
  SPORT_METRIC_SAT,
  SPORT_METRIC_HDOP,
  SPORT_METRIC_PREFIX,
  SPORT_METRIC_LAST
};

#define SPORT_GPS_METRIC_LAT_LON_ID   0x0800
#define SPORT_GPS_METRIC_ALT_ID       0x0820
#define SPORT_GPS_METRIC_SPEED_ID     0x0830
#define SPORT_GPS_METRIC_COG_ID       0x0840
#define SPORT_GPS_METRIC_TS_ID        0x0850
#define SPORT_BEACON_METRIC_STAT_ID   0x0860
#define SPORT_BEACON_METRIC_SAT_ID    0x0870
#define SPORT_BEACON_METRIC_HDOP_ID   0x0880
#define SPORT_BEACON_METRIC_PREFIX_ID 0x0890

class SPortMetric {
  public:
    SPortMetric(SPortMetricType type, uint16_t period, uint16_t ID): type(type), periodMS(period), ID(ID) {}
    void getFrame(char *frame, TinyGPSPlus* gps, Beacon *beacon);
  private:
    SPortMetricType type;
    uint32_t lastSentMS = 0;
    uint32_t value = 0;
    uint16_t periodMS;
    uint16_t ID;
};

class SPort {
  public:
    SPort(Config *c, TinyGPSPlus *gps, Beacon *beacon);
    void readChars();
    //void setValues(uint16)
  private:
    Config *config;
    TinyGPSPlus *gps;
    Beacon *beacon;
    const uart_port_t uartPort = UART_NUM_1;
    uint8_t rxBuffer[RX_BUF_SIZE];
    SPortMetricType metricIndex = SPORT_METRIC_ALT;
    void sendResponse(const char msg[]);
    void sendChar(char c);
    void sendEncodedChar(char c);
    char cksum(const char data[], int start, int len);
//    SPortMetric *getNextMetric();
    bool waitForSPort();
    void lookForCommand();
    void parseCommand(const char *msg, int len);
    //void generateMetricFrame(SPortMetric *metric) {
};
#endif //ifndef __SPort_h