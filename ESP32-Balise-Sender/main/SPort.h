
// vim:et:sts=2:sw=2:si
#ifndef __SPort_h
#define __SPort_h
#include "driver/gpio.h"
#include "driver/uart.h"
#include "Config.h"
#include "TinyGPS++.h"
#include "Beacon.h"

#define RX_BUF_SIZE 1024

enum SPortGPSMetricType {
  SPORT_GPS_METRIC_LAT = 0,
  SPORT_GPS_METRIC_LON, 
  SPORT_GPS_METRIC_ALT,
  SPORT_GPS_METRIC_SPEED,
  SPORT_GPS_METRIC_COG,
  SPORT_GPS_METRIC_DATE,
  SPORT_GPS_METRIC_TIME,
  SPORT_BEACON_METRIC_STAT,
  SPORT_BEACON_METRIC_SAT,
  SPORT_BEACON_METRIC_HDOP,
  SPORT_BEACON_METRIC_PREFIX,
  SPORT_BEACON_METRIC_FRAMES,
  SPORT_GPS_METRIC_LAST
};

#define SPORT_GPS_METRIC_LAT_LON_ID   0x0800
#define SPORT_GPS_METRIC_ALT_ID       0x0820
#define SPORT_GPS_METRIC_SPEED_ID     0x0830
#define SPORT_GPS_METRIC_COG_ID       0x0840
#define SPORT_GPS_METRIC_TS_ID        0x0850
#define SPORT_BEACON_METRIC_STAT_ID   0x5200
#define SPORT_BEACON_METRIC_SAT_ID    0x5210
#define SPORT_BEACON_METRIC_HDOP_ID   0x5220
#define SPORT_BEACON_METRIC_PREFIX_ID 0x5230
#define SPORT_BEACON_METRIC_FRAMES_ID 0x5240

#define SPORT_SENSOR_ID_GPS       0xf2 // 0x12 with CRC aka ID19
#define SPORT_SENSOR_ID_GPS_TX    0x53 // 0x13 with CRC aka ID20

class SPortMetric {
  public:
    SPortMetric(uint8_t type, uint16_t ID, uint16_t period_ms): type(type), ID(ID), periodMs(period_ms) {}
    void getFrame(char *frame, TinyGPSPlus* gps, Beacon *beacon);
    bool isTooSoon();
  private:
    uint8_t type;
    uint16_t ID;
    uint16_t periodMs;
    uint32_t lastSentTS = 0;
    uint32_t convertLatLon(float latLon, bool isLat);
    uint32_t convertDateTime(uint8_t yearOrHour, uint8_t monthOrMinute, uint8_t dayOrSecond, bool isDate);
};

class SPort {
  public:
    SPort(Config *c, TinyGPSPlus *gps, Beacon *beacon);
    uint32_t readChars();
  private:
    Config *config;
    TinyGPSPlus *gps;
    Beacon *beacon;
    const uart_port_t uartPort = UART_NUM_1;
    uint8_t rxBuffer[RX_BUF_SIZE];
    uint8_t lastBeaconMetricIndex = SPORT_GPS_METRIC_LAST - 1;
    void sendTooSoon();
    void sendResponse(const char msg[]);
    void sendChar(char c);
    void sendEncodedChar(char c);
    char cksum(const char data[], int start, int len);
    void parseCommand(const char *msg, int len);
    uint8_t findNextMetricToSendIndex();
};
#endif //ifndef __SPort_h
