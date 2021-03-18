// vim:et:sts=2:sw=2:si
#ifndef __Config_h
#define __Config_h
#include "nvs.h"
#include "driver/gpio.h"

enum GPSModel {
  GPS_MODEL_MOCK = 0,
  GPS_MODEL_L80R,
  GPS_MODEL_BN_220,
  GPS_MODEL_L96_UART
};

class Config {
  public:
    Config();
    void begin();
    void printConfig();
    bool areSwitchesEnabled();
    const char *getAppVersion();
    const char *getSSID();
    const char *getBuilder();
    const char *getVersion();
    const char *getSuffix();
    const char *getPrefix();
    uint8_t getGPSSatThrs();
    uint8_t getGPSHDOPThrs();
    const uint8_t *getMACAddr();
    esp_err_t setBuilder(const uint8_t *newBuilder);
    esp_err_t setVersion(const uint8_t *newVersion);
    esp_err_t setPrefix(const uint8_t *newPrefix);
    esp_err_t setSuffix(const uint8_t *newSuffix);
    esp_err_t setGPSModel(const GPSModel model);
    esp_err_t setGPSSatThrs(uint8_t newThrs);
    esp_err_t setGPSHDOPThrs(uint8_t newThrs);
    esp_err_t resetBuilder();
    esp_err_t resetVersion();
    esp_err_t resetPrefix();
    esp_err_t resetSuffix();
    esp_err_t resetGPSModel();
    esp_err_t resetGPSSatThrs();
    esp_err_t resetGPSHDOPThrs();
    gpio_num_t getGroupMSBPort();
    gpio_num_t getGroupLSBPort();
    gpio_num_t getMassMSBPort();
    gpio_num_t getMassLSBPort();
    gpio_num_t getPPSPort();
    GPSModel getGPSModel();
    static constexpr uint8_t BUILDER_LENGTH = 3;
    static constexpr uint8_t VERSION_LENGTH = 3;
    static constexpr uint8_t PREFIX_LENGTH = 4;
    static constexpr uint8_t SUFFIX_LENGTH = 12;
  private:
    esp_err_t getWriteNVSHandle(nvs_handle_t *handle);
    esp_err_t getFixedStr(nvs_handle_t h, const char*name, char *st, uint8_t len);
    esp_err_t getU8(nvs_handle_t h, const char*name, uint8_t *value);
    esp_err_t setStrValue(const char *name, const char *value, int len);
    esp_err_t setU8Value(const char *name, uint8_t value);
    esp_err_t resetValue(const char *name);
    GPSModel model;
    uint8_t GPSSatThrs, GPSHDOPThrs;
    bool switchesEnabled;
    bool hardcodedSuffixEnabled;
    uint8_t macAddr[6];
    char ssid[32];
    char idBuilder[BUILDER_LENGTH + 1];
    char idVersion[VERSION_LENGTH + 1];
    char idPrefix[PREFIX_LENGTH + 1];
    char idSuffix[SUFFIX_LENGTH + 1];
};
#endif //ifndef __Config_h
