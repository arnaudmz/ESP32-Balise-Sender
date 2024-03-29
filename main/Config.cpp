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
#include "Config.h"
#include "esp_system.h"
#include "esp_ota_ops.h"
//#include "esp_wifi.h"
#include "esp_mac.h"
#include "driver/gpio.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <cstring>
#include "esp_log.h"
static constexpr char TAG[] = "Config";

static_assert(strlen(CONFIG_BEACON_ID_BUILDER) == Config::BUILDER_LENGTH, "BEACON_ID_BUILDER string shoud be 3 char long!");
static_assert(CONFIG_BEACON_ID_BUILDER[3] == 0, "BEACON_ID_BUILDER string shoud be null-terminated!");
static_assert(strlen(CONFIG_BEACON_ID_VERSION) == Config::VERSION_LENGTH, "BEACON_VERSION string shoud be 3 char long!");
static_assert(CONFIG_BEACON_ID_VERSION[3] == 0, "BEACON_VERSION string shoud be null-terminated!");

static constexpr char BUILDER_NVS_NAME[]       = "ovr_builder";
static constexpr char VERSION_NVS_NAME[]       = "ovr_version";
static constexpr char PREFIX_NVS_NAME[]        = "ovr_prefix";
static constexpr char SUFFIX_NVS_NAME[]        = "ovr_suffix";
static constexpr char GPS_MODEL_NVS_NAME[]     = "gps_model";
static constexpr char GPS_SAT_THRS_NVS_NAME[]  = "gps_sat_thrs";
static constexpr char GPS_HDOP_THRS_NVS_NAME[] = "gps_hdop_thrs";
static constexpr char TELEMETRY_NVS_NAME[]     = "telemetry";
static constexpr char NVS_HANDLE_NAME[]        = "beacon";

Config::Config():
model(GPS_MODEL_L80R),
GPSSatThrs(4),
GPSHDOPThrs(40),
#ifdef CONFIG_IDF_TARGET_ESP32
switchesEnabled(true),
#else
switchesEnabled(false),
#endif
hardcodedSuffixEnabled(false),
idBuilder(CONFIG_BEACON_ID_BUILDER),
idVersion(CONFIG_BEACON_ID_VERSION),
idPrefix(""),
idSuffix(""),
telemetry("") {
  nvs_flash_init();
  initSuffixFromMac();
  nvs_handle_t my_nvs_handle;
  esp_err_t err;
  err = nvs_open(NVS_HANDLE_NAME, NVS_READONLY, &my_nvs_handle);
  if (err != ESP_OK) {
    ESP_LOGI(TAG, "No config in NVS (%s), using factory settings", esp_err_to_name(err));
  } else {
    getFixedStr(my_nvs_handle, BUILDER_NVS_NAME, idBuilder, sizeof(idBuilder));
    getFixedStr(my_nvs_handle, VERSION_NVS_NAME, idVersion, sizeof(idVersion));
    getFixedStr(my_nvs_handle, TELEMETRY_NVS_NAME, telemetry, sizeof(telemetry));
    switchesEnabled = (getFixedStr(my_nvs_handle, PREFIX_NVS_NAME, idPrefix, sizeof(idPrefix)) != ESP_OK);
    hardcodedSuffixEnabled = (getFixedStr(my_nvs_handle, SUFFIX_NVS_NAME, idSuffix, sizeof(idSuffix)) == ESP_OK);
    getU8(my_nvs_handle, GPS_MODEL_NVS_NAME, (uint8_t *)&model);
    getU8(my_nvs_handle, GPS_SAT_THRS_NVS_NAME, &GPSSatThrs);
    getU8(my_nvs_handle, GPS_HDOP_THRS_NVS_NAME, &GPSHDOPThrs);
  }
  snprintf(ssid, 32, "%3s_%12s", idBuilder, idSuffix);
}

void Config::initSuffixFromMac() {
  ESP_ERROR_CHECK( esp_read_mac(macAddr, ESP_MAC_WIFI_STA) );
  snprintf(idSuffix, 13, "%02X%02X%02X%02X%02X%02X", macAddr[0], macAddr[1], macAddr[2], macAddr[3], macAddr[4], macAddr[5]);
}

esp_err_t Config::getU8(nvs_handle_t h, const char*name, uint8_t *value) {
  esp_err_t err = nvs_get_u8(h, name, value);
  switch (err) {
    case ESP_OK:
      ESP_LOGI(TAG, "Overriding value from NVS(%s) to %d", name, *value);
      break;
    case ESP_ERR_NVS_NOT_FOUND:
      ESP_LOGI(TAG, "Using Default value for %s (%d)", name, *value);
      break;
    default:
      ESP_LOGE(TAG, "Error getting NVS(%s): %s", name, esp_err_to_name(err));
      break;
  }
  return err;
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
const char *Config::getAppVersion() {
  return esp_app_get_description()->version;
}

GPSModel Config::getGPSModel() {
  return model;
}

uint8_t Config::getGPSSatThrs() {
  return GPSSatThrs;
}

uint8_t Config::getGPSHDOPThrs() {
  return GPSHDOPThrs;
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

const char *Config::getTelemetryModeStr() {
  return telemetry;
}

TelemetryMode Config::getTelemetryMode() {
  if (strcmp(telemetry, "FRSP") == 0) {
    return TELEMETRY_FRSP;
  }
  if (strcmp(telemetry, "JETI") == 0) {
    return TELEMETRY_JETI;
  }
  if (strcmp(telemetry, "JEEX") == 0) {
    return TELEMETRY_JEEX;
  }
  return TELEMETRY_OFF;
}

void Config::printConfig() {
  const esp_app_desc_t *app = esp_app_get_description();
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
    case GPS_MODEL_AT6558:
      ESP_LOGI(TAG, "GPS Model: AT6558 (UART)");
      break;
    default:
      break;
  }
  ESP_LOGI(TAG, "Min GPS Sattelites Count Threshold to set home position: %d", getGPSSatThrs());
  ESP_LOGI(TAG, "Max GPS HDOP Threshold to set home position: %d/10", getGPSHDOPThrs());
  if (switchesEnabled) {
    ESP_LOGI(TAG, "Swiches are enabled:");
    ESP_LOGI(TAG, "  - IO for Group MSB: %d", getGroupMSBPort());
    ESP_LOGI(TAG, "  - IO for Group LSB: %d", getGroupLSBPort());
    ESP_LOGI(TAG, "  - IO for Mass MSB: %d", getMassMSBPort());
    ESP_LOGI(TAG, "  - IO for Mass LSB: %d", getMassLSBPort());
  } else {
    ESP_LOGI(TAG, "Swiches are disabled, %s is hardcoded as prefix", idPrefix);
  }
  switch (getTelemetryMode()) {
    case TELEMETRY_OFF:
      ESP_LOGI(TAG, "Telemetry (%s): OFF", telemetry);
      break;
    case TELEMETRY_FRSP:
      ESP_LOGI(TAG, "Telemetry (%s): FrSky SPort", telemetry);
      break;
    case TELEMETRY_JETI:
      ESP_LOGI(TAG, "Telemetry (%s): Jeti Telemetry", telemetry);
      break;
    case TELEMETRY_JEEX:
      ESP_LOGI(TAG, "Telemetry (%s): Jeti Telemetry Over ExBus", telemetry);
      break;
  }
}

const uint8_t *Config::getMACAddr() {
  return macAddr;
}

esp_err_t Config::setBuilder(const uint8_t *newBuilder) {
  return setStrValue(BUILDER_NVS_NAME, (const char *)newBuilder, BUILDER_LENGTH);
}

esp_err_t Config::setVersion(const uint8_t *newVersion) {
  return setStrValue(VERSION_NVS_NAME, (const char *)newVersion, VERSION_LENGTH);
}

esp_err_t Config::setPrefix(const uint8_t *newPrefix) {
  if (memcmp(idPrefix, newPrefix, PREFIX_LENGTH) == 0) {
      ESP_LOGI(TAG, "Ignoring new prefix, unchanged");
      return ESP_OK;
  }
  memcpy(idPrefix, newPrefix, PREFIX_LENGTH);
  idPrefix[PREFIX_LENGTH] = '\0';
  switchesEnabled = false;
  return setStrValue(PREFIX_NVS_NAME, (const char *)newPrefix, PREFIX_LENGTH);
}

esp_err_t Config::setSuffix(const uint8_t *newSuffix) {
  return setStrValue(SUFFIX_NVS_NAME, (const char *)newSuffix, SUFFIX_LENGTH);
}

esp_err_t Config::setGPSModel(const GPSModel model) {
  return setU8Value(GPS_MODEL_NVS_NAME, (uint8_t) model);
}

esp_err_t Config::setGPSSatThrs(uint8_t value) {
  return setU8Value(GPS_SAT_THRS_NVS_NAME, value);
}

esp_err_t Config::setGPSHDOPThrs(uint8_t value) {
  return setU8Value(GPS_HDOP_THRS_NVS_NAME, value);
}

esp_err_t Config::setTelemetryMode(const uint8_t *newTelemetryMode) {
  return setStrValue(TELEMETRY_NVS_NAME, (const char *)newTelemetryMode, TELEMETRY_LENGTH);
}

esp_err_t Config::resetBuilder() {
  strncpy(idBuilder, CONFIG_BEACON_ID_BUILDER, BUILDER_LENGTH + 1);
  return resetValue(BUILDER_NVS_NAME);
}

esp_err_t Config::resetVersion() {
  strncpy(idVersion, CONFIG_BEACON_ID_VERSION, VERSION_LENGTH + 1);
  return resetValue(VERSION_NVS_NAME);
}

esp_err_t Config::resetPrefix() {
  return resetValue(PREFIX_NVS_NAME);
}

esp_err_t Config::resetSuffix() {
  initSuffixFromMac();
  return resetValue(SUFFIX_NVS_NAME);
}

esp_err_t Config::resetGPSModel() {
  model = GPS_MODEL_L80R;
  return resetValue(GPS_MODEL_NVS_NAME);
}

esp_err_t Config::resetGPSSatThrs() {
  return resetValue(GPS_SAT_THRS_NVS_NAME);
}

esp_err_t Config::resetGPSHDOPThrs() {
  return resetValue(GPS_HDOP_THRS_NVS_NAME);
}

esp_err_t Config::resetTelemetryMode() {
  return resetValue(TELEMETRY_NVS_NAME);
}

esp_err_t Config::resetValue(const char *name) {
  nvs_handle_t handle;
  esp_err_t err = getWriteNVSHandle(&handle);
  if(err != ESP_OK) {
    ESP_LOGE(TAG, "Error getting NVS write handle: %s", esp_err_to_name(err));
    return err;
  }
  ESP_LOGI(TAG, "Resetting conf in NVS for %s", name);
  err = nvs_erase_key(handle, name);
  nvs_close(handle);
  return err;
}

esp_err_t Config::getWriteNVSHandle(nvs_handle_t *handle) {
  return nvs_open(NVS_HANDLE_NAME, NVS_READWRITE, handle);
}

esp_err_t Config::setStrValue(const char *name, const char *value, int len) {
  if (len > 15) {
    return ESP_ERR_NVS_VALUE_TOO_LONG;
  }
  nvs_handle_t handle;
  esp_err_t err = getWriteNVSHandle(&handle);
  if(err != ESP_OK) {
    ESP_LOGE(TAG, "Error getting NVS write handle: %s", esp_err_to_name(err));
    return err;
  }
  char st[16];
  memcpy(st, value, len);
  st[len] = '\0';
  ESP_LOGI(TAG, "Setting conf in NVS for %s to %s (%d bytes)", name, st, len);
  err = nvs_set_str(handle, name, st);
  nvs_close(handle);
  return err;
}

esp_err_t Config::setU8Value(const char *name, uint8_t value) {
  nvs_handle_t handle;
  esp_err_t err = getWriteNVSHandle(&handle);
  if(err != ESP_OK) {
    ESP_LOGE(TAG, "Error getting NVS write handle: %s", esp_err_to_name(err));
    return err;
  }
  ESP_LOGI(TAG, "Setting conf in NVS for %s to %d", name, value);
  err = nvs_set_u8(handle, name, value);
  nvs_close(handle);
  return err;
}
