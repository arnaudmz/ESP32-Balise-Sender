// vim:et:sts=2:sw=2:si
#include "Config.h"
#include "esp_ota_ops.h"
#include "esp_wifi.h"
#include "driver/gpio.h"
#include "nvs.h"
#include <cstring>
//#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
//#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#define LOG_LOCAL_LEVEL ESP_LOG_INFO
static constexpr char TAG[] = "Config";
#include "esp_log.h"

#define GROUP_MSB_IO (gpio_num_t)CONFIG_BEACON_ID_GROUP_MSB_IO
#define GROUP_LSB_IO (gpio_num_t)CONFIG_BEACON_ID_GROUP_LSB_IO
#define MASS_MSB_IO  (gpio_num_t)CONFIG_BEACON_ID_MASS_MSB_IO
#define MASS_LSB_IO  (gpio_num_t)CONFIG_BEACON_ID_MASS_LSB_IO


Config::Config():
  switches_enabled(true),
  hardcoded_suffix_enabled(false),
  id_builder(CONFIG_BEACON_ID_BUILDER),
  id_version(CONFIG_BEACON_ID_VERSION),
  id_prefix(""),
  id_suffix("")
{

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

void Config::begin(const uint8_t* mac) {
  memcpy(mac_addr, mac, 6);
  nvs_handle_t my_nvs_handle;
  esp_err_t err;
  snprintf(id_suffix, 13, "%02X%02X%02X%02X%02X%02X", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  err = nvs_open("beacon", NVS_READONLY, &my_nvs_handle);
  if (err != ESP_OK) {
    ESP_LOGI(TAG, "No config in NVS (%s), using factory settings", esp_err_to_name(err));
    return;
  }
  getFixedStr(my_nvs_handle, "ovr_builder", id_builder, sizeof(id_builder));
  getFixedStr(my_nvs_handle, "ovr_version", id_version, sizeof(id_version));
  switches_enabled = (getFixedStr(my_nvs_handle, "ovr_prefix", id_prefix, sizeof(id_prefix)) != ESP_OK);
  hardcoded_suffix_enabled = (getFixedStr(my_nvs_handle, "ovr_suffix", id_suffix, sizeof(id_suffix)) == ESP_OK);
}


bool Config::areSwitchesEnabled() {
  return switches_enabled;
}

const char *Config::getSSID() {
  return ssid;
}

const char *Config::getBuilder() {
  return id_builder;
}

const char *Config::getVersion() {
  return id_version;
}

const char *Config::getSuffix() {
  return id_suffix;
}

const char *Config::getPrefix() {
  return id_prefix;
}

void Config::printConfig() {
  const esp_app_desc_t *app = esp_ota_get_app_description();
  ESP_LOGI(TAG, "Starting Beacon (%s) version %s", app->project_name, app->version);
  ESP_LOGI(TAG, "MAC address: %02X:%02X:%02X:%02X:%02X:%02X", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  ESP_LOGI(TAG, "SSID: %s", ssid);
  ESP_LOGI(TAG, "ID Builder: %s", id_builder);
  ESP_LOGI(TAG, "ID Version: %s", id_version);
  ESP_LOGI(TAG, "ID Suffix: %s (%s)", id_suffix, hardcoded_suffix_enabled ? "Hardcoded": "from MAC");
#ifdef CONFIG_BEACON_GPS_MOCK
  ESP_LOGI(TAG, "GPS Model: mock");
#endif
#ifdef CONFIG_BEACON_GPS_L96_I2C
  ESP_LOGI(TAG, "GPS Model: L96 (I2C)");
  ESP_LOGI(TAG, "IO for SCL: %d", CONFIG_BEACON_GPS_SCL_IO);
  ESP_LOGI(TAG, "IO for SDA: %d", CONFIG_BEACON_GPS_SDA_IO);
#endif
#ifdef CONFIG_BEACON_GPS_L96_UART
  ESP_LOGI(TAG, "GPS Model: L96 (UART + PPS)");
  ESP_LOGI(TAG, "IO for PPS: %d", PPS_IO);
#endif
#ifdef CONFIG_BEACON_GPS_L80R_UART
  ESP_LOGI(TAG, "GPS Model: L80R (UART + PPS)");
  ESP_LOGI(TAG, "IO for PPS: %d", PPS_IO);
#endif
#ifdef CONFIG_BEACON_GPS_BN_220_UART
  ESP_LOGI(TAG, "GPS Model: BN-220 (UART)");
#endif
  if (switches_enabled) {
    ESP_LOGI(TAG, "Swiches are enabled.");
    ESP_LOGI(TAG, "  - IO for Group MSB: %d", GROUP_MSB_IO);
    ESP_LOGI(TAG, "  - IO for Group LSB: %d", GROUP_LSB_IO);
    ESP_LOGI(TAG, "  - IO for Mass MSB: %d", MASS_MSB_IO);
    ESP_LOGI(TAG, "  - IO for Mass LSB: %d", MASS_LSB_IO);
  } else {
    ESP_LOGI(TAG, "Swiches are disabled, %s is hardcoded as prefix", id_prefix);
  }
}
