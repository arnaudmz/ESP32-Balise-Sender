// vim:et:sts=2:sw=2:si
#ifndef __Config_h
#define __Config_h
#include "nvs.h"
#include "driver/gpio.h"

class Config {
  public:
    Config();
    void begin();
    void printConfig();
    bool areSwitchesEnabled();
    const char *getSSID();
    const char *getBuilder();
    const char *getVersion();
    const char *getSuffix();
    const char *getPrefix();
    const uint8_t *getMACAddr();
    gpio_num_t getGroupMSBPort();
    gpio_num_t getGroupLSBPort();
    gpio_num_t getMassMSBPort();
    gpio_num_t getMassLSBPort();
#if defined(CONFIG_BEACON_GPS_L80R_UART) || defined(CONFIG_BEACON_GPS_L96_UART)
    gpio_num_t getPPSPort();
#endif //#if defined(CONFIG_BEACON_GPS_L80R_UART) || defined(CONFIG_BEACON_GPS_L96_UART)
  private:
    esp_err_t getFixedStr(nvs_handle_t h, const char*name, char *st, uint8_t len);
    bool switchesEnabled;
    bool hardcodedSuffixEnabled;
    uint8_t macAddr[6];
    char ssid[32];
    char idBuilder[4];
    char idVersion[4];
    char idPrefix[5];
    char idSuffix[13];
};
#endif //ifndef __Config_h
