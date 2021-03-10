// vim:et:sts=2:sw=2:si
#ifndef __Config_h
#define __Config_h
#include "nvs.h"

class Config {
  public:
    Config();
    void begin(const uint8_t *mac);
    void printConfig();
    bool areSwitchesEnabled();
    const char *getSSID();
    const char *getBuilder();
    const char *getVersion();
    const char *getSuffix();
    const char *getPrefix();
    uint8_t getSSIDSize();
  private:
    esp_err_t getFixedStr(nvs_handle_t h, const char*name, char *st, uint8_t len);
    bool switches_enabled;
    bool hardcoded_suffix_enabled;
    uint8_t mac_addr[6];
    char ssid[32];
    char id_builder[4];
    char id_version[4];
    char id_prefix[5];
    char id_suffix[13];
};
#endif //ifndef __Config_h
