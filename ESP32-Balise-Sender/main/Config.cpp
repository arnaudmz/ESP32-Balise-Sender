// vim:et:sts=2:sw=2:si
#include "Config.h"
#include "esp_ota_ops.h"
#include "esp_wifi.h"
#include "driver/gpio.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <cstring>
//#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
//#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#define LOG_LOCAL_LEVEL ESP_LOG_INFO
static constexpr char TAG[] = "Config";
#include "esp_log.h"

static_assert(strlen(CONFIG_BEACON_ID_BUILDER) == 3, "BEACON_ID_BUILDER string shoud be 3 char long!");
static_assert(CONFIG_BEACON_ID_BUILDER[3] == 0, "BEACON_ID_BUILDER string shoud be null-terminated!");
static_assert(strlen(CONFIG_BEACON_ID_VERSION) == 3, "BEACON_VERSION string shoud be 3 char long!");
static_assert(CONFIG_BEACON_ID_VERSION[3] == 0, "BEACON_VERSION string shoud be null-terminated!");

Config::Config():
model(GPS_MODEL_L80R),
switchesEnabled(true),
hardcodedSuffixEnabled(false),
idBuilder(CONFIG_BEACON_ID_BUILDER),
idVersion(CONFIG_BEACON_ID_VERSION),
idPrefix(""),
idSuffix("") {
  nvs_flash_init();
  ESP_ERROR_CHECK( esp_read_mac(macAddr, ESP_MAC_WIFI_STA) );
  nvs_handle_t my_nvs_handle;
  esp_err_t err;
  snprintf(idSuffix, 13, "%02X%02X%02X%02X%02X%02X", macAddr[0], macAddr[1], macAddr[2], macAddr[3], macAddr[4], macAddr[5]);
  err = nvs_open("beacon", NVS_READONLY, &my_nvs_handle);
  if (err != ESP_OK) {
    ESP_LOGI(TAG, "No config in NVS (%s), using factory settings", esp_err_to_name(err));
  } else {
    getFixedStr(my_nvs_handle, "ovr_builder", idBuilder, sizeof(idBuilder));
    getFixedStr(my_nvs_handle, "ovr_version", idVersion, sizeof(idVersion));
    switchesEnabled = (getFixedStr(my_nvs_handle, "ovr_prefix", idPrefix, sizeof(idPrefix)) != ESP_OK);
    hardcodedSuffixEnabled = (getFixedStr(my_nvs_handle, "ovr_suffix", idSuffix, sizeof(idSuffix)) == ESP_OK);
    err = nvs_get_u8(my_nvs_handle, "gps_model", (uint8_t *)&model);
    switch (err) {
      case ESP_OK:
        ESP_LOGI(TAG, "Overriding GPS Model to %d", model);
        break;
      case ESP_ERR_NVS_NOT_FOUND:
        ESP_LOGI(TAG, "Using Default GPS Model (%d)", model);
        break;
      default:
        ESP_LOGE(TAG, "Error getting model: %s", esp_err_to_name(err));
    }
  }
  snprintf(ssid, 32, "%3s_%12s", idBuilder, idSuffix);
}

esp_err_t Config::getFixedStr(nvs_handle_t h, const char*name, char *st, uint8_t len) {
  char tmp_str[16];
  size_t size;
  esp_err_t err = nvs_get_str(h, name, NULL, &size);
  if (err != ESP_OK) {
    ESP_LOGD(TAG, "Error (%s), getting %s size", esp_err_to_name(err), name);
    return err;
  }
  if (size != len) {
    ESP_LOGD(TAG, "Ignoring %s: sizes don't match (%d!=%d)", name, size, len);
    return ESP_ERR_NVS_INVALID_LENGTH;
  }
  err = nvs_get_str(h, name, tmp_str, &size);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Error (%s), getting %s content", esp_err_to_name(err), name);
    return err;
  }
  strncpy(st, tmp_str, size);
  ESP_LOGD(TAG, "Read %s=%s from nvs", name, st);
  return ESP_OK;
}

GPSModel Config::getGPSModel() {
  return model;
}

bool Config::areSwitchesEnabled() {
  return switchesEnabled;
}

const char *Config::getSSID() {
  return ssid;
}

const char *Config::getBuilder() {
  return idBuilder;
}

const char *Config::getVersion() {
  return idVersion;
}

const char *Config::getSuffix() {
  return idSuffix;
}

const char *Config::getPrefix() {
  return idPrefix;
}

gpio_num_t Config::getGroupMSBPort() {
  return (gpio_num_t)CONFIG_BEACON_ID_GROUP_MSB_IO;
}

gpio_num_t Config::getGroupLSBPort() {
  return (gpio_num_t)CONFIG_BEACON_ID_GROUP_LSB_IO;
}

gpio_num_t Config::getMassMSBPort() {
  return (gpio_num_t)CONFIG_BEACON_ID_MASS_MSB_IO;
}

gpio_num_t Config::getMassLSBPort() {
  return (gpio_num_t)CONFIG_BEACON_ID_MASS_LSB_IO;
}

gpio_num_t Config::getPPSPort() {
  return (gpio_num_t)CONFIG_BEACON_GPS_PPS_IO;
}

void Config::printConfig() {
  const esp_app_desc_t *app = esp_ota_get_app_description();
  ESP_LOGI(TAG, "Starting Beacon (%s) version %s", app->project_name, app->version);
  ESP_LOGI(TAG, "MAC address: %02X:%02X:%02X:%02X:%02X:%02X", macAddr[0], macAddr[1], macAddr[2], macAddr[3], macAddr[4], macAddr[5]);
  ESP_LOGI(TAG, "SSID: %s", ssid);
  ESP_LOGI(TAG, "ID Builder: %s", idBuilder);
  ESP_LOGI(TAG, "ID Version: %s", idVersion);
  ESP_LOGI(TAG, "ID Suffix: %s (%s)", idSuffix, hardcodedSuffixEnabled ? "Hardcoded": "from MAC");
  switch (model) {
    case GPS_MODEL_MOCK:
      ESP_LOGI(TAG, "GPS Model: mock");
      break;
    case GPS_MODEL_L96_UART:
      ESP_LOGI(TAG, "GPS Model: L96 (UART + PPS)");
      ESP_LOGI(TAG, "IO for PPS: %d", getPPSPort());
      break;
    case GPS_MODEL_L80R:
      ESP_LOGI(TAG, "GPS Model: L80R (UART + PPS)");
      ESP_LOGI(TAG, "IO for PPS: %d", getPPSPort());
      break;
    case GPS_MODEL_BN_220:
      ESP_LOGI(TAG, "GPS Model: BN-220 (UART)");
      break;
  }
  if (switchesEnabled) {
    ESP_LOGI(TAG, "Swiches are enabled:");
    ESP_LOGI(TAG, "  - IO for Group MSB: %d", getGroupMSBPort());
    ESP_LOGI(TAG, "  - IO for Group LSB: %d", getGroupLSBPort());
    ESP_LOGI(TAG, "  - IO for Mass MSB: %d", getMassMSBPort());
    ESP_LOGI(TAG, "  - IO for Mass LSB: %d", getMassLSBPort());
  } else {
    ESP_LOGI(TAG, "Swiches are disabled, %s is hardcoded as prefix", idPrefix);
  }
}

const uint8_t *Config::getMACAddr() {
  return macAddr;
}