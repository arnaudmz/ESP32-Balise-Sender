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

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include <string.h>
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"

#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
//#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
//#define LOG_LOCAL_LEVEL ESP_LOG_INFO
static constexpr char TAG[] = "BLEConfig";
#include "esp_log.h"

#include "Config.h"
#include "LED.h"

#define PROFILE_NUM                 1
#define PROFILE_APP_IDX             0
#define ESP_APP_ID                  0x55
#define DEVICE_NAME                 "LaBalise"
#define SVC_INST_ID                 0

#define PREPARE_BUF_MAX_SIZE        1024
#define CHAR_DECLARATION_SIZE       (sizeof(uint8_t))

#define ADV_CONFIG_FLAG             (1 << 0)
#define SCAN_RSP_CONFIG_FLAG        (1 << 1)

enum {
  IDX_SVC,
  IDX_BUILD_VERSION,
  IDX_BUILD_VERSION_VAL,
  IDX_GPS_MODEL,
  IDX_GPS_MODEL_VAL,
  IDX_GPS_SAT_THRS,
  IDX_GPS_SAT_THRS_VAL,
  IDX_GPS_HDOP_THRS,
  IDX_GPS_HDOP_THRS_VAL,
  IDX_BUILDER,
  IDX_BUILDER_VAL,
  IDX_VERSION,
  IDX_VERSION_VAL,
  IDX_PREFIX,
  IDX_PREFIX_VAL,
  IDX_SUFFIX,
  IDX_SUFFIX_VAL,
  IDX_TELEMETRY,
  IDX_TELEMETRY_VAL,
  HRS_IDX_NB,
};

Config *my_config;

static uint8_t adv_config_done       = 0;

uint16_t handle_table[HRS_IDX_NB];

static uint8_t service_uuid[16] = {
  /* LSB <--------------------------------------------------------------------------------> MSB */
  //first uuid, 16bit, [12],[13] is the value
  0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00,
};

/* The length of adv data must be less than 31 bytes */
static esp_ble_adv_data_t adv_data = {
  .set_scan_rsp        = false,
  .include_name        = true,
  .include_txpower     = true,
  .min_interval        = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
  .max_interval        = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
  .appearance          = 0x00,
  .manufacturer_len    = 0,    //TEST_MANUFACTURER_DATA_LEN,
  .p_manufacturer_data = NULL, //test_manufacturer,
  .service_data_len    = 0,
  .p_service_data      = NULL,
  .service_uuid_len    = sizeof(service_uuid),
  .p_service_uuid      = service_uuid,
  .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

// scan response data
static esp_ble_adv_data_t scan_rsp_data = {
  .set_scan_rsp        = true,
  .include_name        = true,
  .include_txpower     = true,
  .min_interval        = 0x0006,
  .max_interval        = 0x0010,
  .appearance          = 0x00,
  .manufacturer_len    = 0, //TEST_MANUFACTURER_DATA_LEN,
  .p_manufacturer_data = NULL, //&test_manufacturer[0],
  .service_data_len    = 0,
  .p_service_data      = NULL,
  .service_uuid_len    = sizeof(service_uuid),
  .p_service_uuid      = service_uuid,
  .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t adv_params = {
  .adv_int_min         = 0x20,
  .adv_int_max         = 0x40,
  .adv_type            = ADV_TYPE_IND,
  .own_addr_type       = BLE_ADDR_TYPE_PUBLIC,
  .channel_map         = ADV_CHNL_ALL,
  .adv_filter_policy   = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

struct gatts_profile_inst {
  esp_gatts_cb_t gatts_cb;
  uint16_t gatts_if;
  uint16_t app_id;
  uint16_t conn_id;
  uint16_t service_handle;
  esp_gatt_srvc_id_t service_id;
  uint16_t char_handle;
  esp_bt_uuid_t char_uuid;
  esp_gatt_perm_t perm;
  esp_gatt_char_prop_t property;
  uint16_t descr_handle;
  esp_bt_uuid_t descr_uuid;
};

static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
    esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst profile_tab[PROFILE_NUM] = {
  [PROFILE_APP_IDX] = {
    .gatts_cb = gatts_profile_event_handler,
    .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
  },
};

/* Service */
static const uint16_t GATTS_SERVICE_UUID            = 0x414D;
static const uint16_t GATTS_CHAR_UUID_BUILD_VERSION = 0x0001;
static const uint16_t GATTS_CHAR_UUID_GPS_MODEL     = 0x1234;
static const uint16_t GATTS_CHAR_UUID_GPS_SAT_THRS  = 0x0007;
static const uint16_t GATTS_CHAR_UUID_GPS_HDOP_THRS = 0x0025;
static const uint16_t GATTS_CHAR_UUID_BUILDER       = 0x0BBB;
static const uint16_t GATTS_CHAR_UUID_VERSION       = 0x0111;
static const uint16_t GATTS_CHAR_UUID_PREFIX        = 0x1002;
static const uint16_t GATTS_CHAR_UUID_SUFFIX        = 0xFFFF;
static const uint16_t GATTS_CHAR_UUID_TELEMETRY     = 0x0033;

static const uint16_t primary_service_uuid         = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid   = ESP_GATT_UUID_CHAR_DECLARE;
static const uint8_t char_prop_read_only   = ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t char_prop_read_write   = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ;
static uint8_t gps_model;
static uint8_t gps_sat_thrs;
static uint8_t gps_hdop_thrs;
static uint8_t app_version[32];
static uint8_t builder[Config::BUILDER_LENGTH];
static uint8_t version[Config::VERSION_LENGTH];
static uint8_t prefix[Config::PREFIX_LENGTH] = {'\0', '\0', '\0', '\0'};
static uint8_t suffix[Config::SUFFIX_LENGTH];
static uint8_t telemetry[Config::TELEMETRY_LENGTH];

/* Full Database Description - Used to add attributes into the database */
static const esp_gatts_attr_db_t gatt_db[HRS_IDX_NB] = {
  // Service Declaration
  [IDX_SVC]        =
  {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
                          sizeof(uint16_t), sizeof(GATTS_SERVICE_UUID), (uint8_t *)&GATTS_SERVICE_UUID}},
  /* Characteristic Declaration */
  [IDX_BUILD_VERSION]      =
  {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
                          CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_only}},
  /* Characteristic Value */
  [IDX_BUILD_VERSION_VAL]  =
  {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_BUILD_VERSION, ESP_GATT_PERM_READ,
                          sizeof(app_version), sizeof(app_version), (uint8_t *)app_version}},
  /* Characteristic Declaration */
  [IDX_GPS_MODEL]      =
  {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
                          CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},
  /* Characteristic Value */
  [IDX_GPS_MODEL_VAL]  =
  {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_GPS_MODEL, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                          sizeof(gps_model), sizeof(gps_model), (uint8_t *)&gps_model}},
  /* Characteristic Declaration */
  [IDX_GPS_SAT_THRS]      =
  {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
                          CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},
  /* Characteristic Value */
  [IDX_GPS_SAT_THRS_VAL]  =
  {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_GPS_SAT_THRS, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                          sizeof(gps_sat_thrs), sizeof(gps_sat_thrs), (uint8_t *)&gps_sat_thrs}},
  /* Characteristic Declaration */
  [IDX_GPS_HDOP_THRS]      =
  {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
                          CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},
  /* Characteristic Value */
  [IDX_GPS_HDOP_THRS_VAL]  =
  {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_GPS_HDOP_THRS, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                          sizeof(gps_hdop_thrs), sizeof(gps_hdop_thrs), (uint8_t *)&gps_hdop_thrs}},
  /* Characteristic Declaration */
  [IDX_BUILDER]      =
  {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
                          CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},
  /* Characteristic Value */
  [IDX_BUILDER_VAL]  =
  {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_BUILDER, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                          sizeof(builder), sizeof(builder), (uint8_t *)builder}},
  /* Characteristic Declaration */
  [IDX_VERSION]      =
  {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
                          CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},
  /* Characteristic Value */
  [IDX_VERSION_VAL]  =
  {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_VERSION, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                          sizeof(version), sizeof(version), (uint8_t *)version}},
  /* Characteristic Declaration */
  [IDX_PREFIX]      =
  {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
                          CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},
  /* Characteristic Value */
  [IDX_PREFIX_VAL]  =
  {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_PREFIX, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                          sizeof(prefix), sizeof(prefix), (uint8_t *)prefix}},
  /* Characteristic Declaration */
  [IDX_SUFFIX]      =
  {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
                          CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},
  /* Characteristic Value */
  [IDX_SUFFIX_VAL]  =
  {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_SUFFIX, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                          sizeof(suffix), sizeof(suffix), (uint8_t *)suffix}},
  /* Characteristic Declaration */
  [IDX_TELEMETRY]      =
  {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
                          CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},
  /* Characteristic Value */
  [IDX_TELEMETRY_VAL]  =
  {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TELEMETRY, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                          sizeof(telemetry), sizeof(telemetry), (uint8_t *)telemetry}},
};

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
  switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
      adv_config_done &= (~ADV_CONFIG_FLAG);
      if (adv_config_done == 0) {
        esp_ble_gap_start_advertising(&adv_params);
      }
      break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
      adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
      if (adv_config_done == 0) {
        esp_ble_gap_start_advertising(&adv_params);
      }
      break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
      /* advertising start complete event to indicate advertising start successfully or failed */
      if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
        ESP_LOGE(TAG, "advertising start failed");
      } else {
        ESP_LOGI(TAG, "advertising start successfully");
      }
      break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
      if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
        ESP_LOGE(TAG, "Advertising stop failed");
      } else {
        ESP_LOGI(TAG, "Stop adv successfully\n");
      }
      break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
      ESP_LOGI(TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
          param->update_conn_params.status,
          param->update_conn_params.min_int,
          param->update_conn_params.max_int,
          param->update_conn_params.conn_int,
          param->update_conn_params.latency,
          param->update_conn_params.timeout);
      break;
    default:
      break;
  }
}

bool check_data_are_chars(uint8_t *v, uint8_t len) {
  for (int i = 0; i < len; i++) {
    if (v[i] < '0' || v[i] > 'Z' || v[i] == '@') {
      return false;
    }
  }
  return true;
}

void handle_write(uint16_t handle, uint8_t *value, uint16_t len) {
  if (handle == handle_table[IDX_BUILDER_VAL]) {
    if (len != Config::BUILDER_LENGTH) {
      ESP_LOGD(TAG, "Reset Builder to factory setting");
      my_config->resetBuilder();
    } else {
      if (check_data_are_chars(value, len)) {
        ESP_LOGD(TAG, "Builder written");
        my_config->setBuilder(value);
      } else {
        ESP_LOGD(TAG, "Builder not updated, contains invalid chars");
      }
    }
  } else if (handle == handle_table[IDX_VERSION_VAL]) {
    if (len != Config::VERSION_LENGTH) {
      ESP_LOGD(TAG, "Reset Version to factory setting");
      my_config->resetVersion();
    } else {
      if (check_data_are_chars(value, len)) {
        ESP_LOGD(TAG, "Version written");
        my_config->setVersion(value);
      } else {
        ESP_LOGD(TAG, "Version not updated, contains invalid chars");
      }
    }
  } else if (handle == handle_table[IDX_PREFIX_VAL]) {
    if (len != Config::PREFIX_LENGTH) {
      ESP_LOGD(TAG, "Reset Prefix to factory setting");
      my_config->resetPrefix();
    } else {
      if (check_data_are_chars(value, len)) {
        ESP_LOGD(TAG, "Prefix written");
        my_config->setPrefix(value);
      } else {
        ESP_LOGD(TAG, "Prefix not updated, contains invalid chars");
      }
    }
  } else if (handle == handle_table[IDX_SUFFIX_VAL]) {
    if (len != Config::SUFFIX_LENGTH) {
      ESP_LOGD(TAG, "Reset Suffix to factory setting");
      my_config->resetSuffix();
    } else {
      if (check_data_are_chars(value, len)) {
        ESP_LOGD(TAG, "Suffix written");
        my_config->setSuffix(value);
      } else {
        ESP_LOGD(TAG, "Suffix not updated, contains invalid chars");
      }
    }
  } else if (handle == handle_table[IDX_TELEMETRY_VAL]) {
    if (len != Config::TELEMETRY_LENGTH) {
      ESP_LOGD(TAG, "Reset Telemetry Mode to factory setting");
      my_config->resetTelemetryMode();
    } else {
      if (check_data_are_chars(value, len)) {
        ESP_LOGD(TAG, "Telemetry Mode written");
        my_config->setTelemetryMode(value);
      } else {
        ESP_LOGD(TAG, "Telemetry mode not updated, contains invalid chars");
      }
    }
  } else if (handle == handle_table[IDX_GPS_MODEL_VAL]) {
    if(*value > GPS_MODEL_L96_UART) {
      ESP_LOGD(TAG, "Reset Model to factory setting");
      my_config->resetGPSModel();
    } else {
        ESP_LOGD(TAG, "Model written");
      my_config->setGPSModel((GPSModel) *value);
    }
  } else if (handle == handle_table[IDX_GPS_SAT_THRS_VAL]) {
    if(*value > 16) {
      ESP_LOGD(TAG, "Reset Sat thrs to factory setting");
      my_config->resetGPSSatThrs();
    } else {
        ESP_LOGD(TAG, "Sat Thrs written");
      my_config->setGPSSatThrs(*value);
    }
  } else if (handle == handle_table[IDX_GPS_HDOP_THRS_VAL]) {
    if(*value > 100 || *value < 10) {
      ESP_LOGD(TAG, "Reset HDOP thrs to factory setting");
      my_config->resetGPSHDOPThrs();
    } else {
        ESP_LOGD(TAG, "HDOP Thrs written");
      my_config->setGPSHDOPThrs(*value);
    }
  } else {
      ESP_LOGD(TAG, "Unkown handle: %d", handle);
  }
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
  switch (event) {
    case ESP_GATTS_REG_EVT:
      ESP_ERROR_CHECK( esp_ble_gap_set_device_name(DEVICE_NAME) );
      //config adv data
      ESP_ERROR_CHECK( esp_ble_gap_config_adv_data(&adv_data) );
      adv_config_done |= ADV_CONFIG_FLAG;
      //config scan response data
      ESP_ERROR_CHECK(esp_ble_gap_config_adv_data(&scan_rsp_data) );
      adv_config_done |= SCAN_RSP_CONFIG_FLAG;
      ESP_ERROR_CHECK( esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, HRS_IDX_NB, SVC_INST_ID) );
      break;
    case ESP_GATTS_WRITE_EVT:
      ESP_LOGD(TAG, "GATT_WRITE_EVT");
      if (!param->write.is_prep) {
        ESP_LOGD(TAG, "GATT_WRITE_EVT, handle = %d, value len = %d, value :", param->write.handle, param->write.len);
        esp_log_buffer_hex(TAG, param->write.value, param->write.len);
        handle_write(param->write.handle, param->write.value, param->write.len);
        /* send response when param->write.need_rsp is true*/
        if (param->write.need_rsp) {
          ESP_LOGD(TAG, "GATT_WRITE_EVT, sending response");
          esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
        }
      } else {
        /* handle prepare write */
        ESP_LOGE(TAG, "GATT_WRITE_EVT, PREP not implemented");
      }
      break;
    case ESP_GATTS_EXEC_WRITE_EVT: 
      ESP_LOGE(TAG, "ESP_GATTS_EXEC_WRITE_EVT, PREP not implemented");
      break;
    case ESP_GATTS_CONNECT_EVT:
      ESP_LOGD(TAG, "ESP_GATTS_CONNECT_EVT, conn_id = %d", param->connect.conn_id);
      esp_log_buffer_hex(TAG, param->connect.remote_bda, 6);
      esp_ble_conn_update_params_t conn_params; // = {0};
      memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
      /* For the iOS system, please refer to Apple official documents about the BLE connection parameters restrictions. */
      conn_params.latency = 0;
      conn_params.max_int = 0x20;    // max_int = 0x20*1.25ms = 40ms
      conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
      conn_params.timeout = 400;    // timeout = 400*10ms = 4000ms
      //start sent the update connection parameters to the peer device.
      esp_ble_gap_update_conn_params(&conn_params);
      break;
    case ESP_GATTS_DISCONNECT_EVT:
      ESP_LOGD(TAG, "ESP_GATTS_DISCONNECT_EVT, reason = 0x%x", param->disconnect.reason);
      esp_ble_gap_start_advertising(&adv_params);
      break;
    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
      if (param->add_attr_tab.status != ESP_GATT_OK) {
        ESP_LOGE(TAG, "create attribute table failed, error code=0x%x", param->add_attr_tab.status);
      } else if (param->add_attr_tab.num_handle != HRS_IDX_NB) {
        ESP_LOGE(TAG, "create attribute table abnormally, num_handle (%d) \
            doesn't equal to HRS_IDX_NB(%d)", param->add_attr_tab.num_handle, HRS_IDX_NB);
      } else {
        ESP_LOGD(TAG, "create attribute table successfully, the number handle = %d\n",param->add_attr_tab.num_handle);
        memcpy(handle_table, param->add_attr_tab.handles, sizeof(handle_table));
        esp_ble_gatts_start_service(handle_table[IDX_SVC]);
      }
      break;
    default:
      break;
  }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
  /* If event is register event, store the gatts_if for each profile */
  if (event == ESP_GATTS_REG_EVT) {
    if (param->reg.status == ESP_GATT_OK) {
      profile_tab[PROFILE_APP_IDX].gatts_if = gatts_if;
    } else {
      ESP_LOGE(TAG, "reg app failed, app_id %04x, status %d",
          param->reg.app_id,
          param->reg.status);
      return;
    }
  }
  for (int idx = 0; idx < PROFILE_NUM; idx++) {
    /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
    if (gatts_if == ESP_GATT_IF_NONE || gatts_if == profile_tab[idx].gatts_if) {
      if (profile_tab[idx].gatts_cb) {
        profile_tab[idx].gatts_cb(event, gatts_if, param);
      }
    }
  }
}

void ble_serve(Config *config, LED *led) {
  my_config = config;
  const char *_app_version = config->getAppVersion();
  led->blinkFastForever();
  memcpy(builder, config->getBuilder(), Config::BUILDER_LENGTH);
  memcpy(version, config->getVersion(), Config::VERSION_LENGTH);
  memcpy(suffix, config->getSuffix(), Config::SUFFIX_LENGTH);
  memcpy(telemetry, config->getTelemetryModeStr(), Config::TELEMETRY_LENGTH);
  memcpy(app_version, _app_version, strlen(_app_version));
  gps_model = config->getGPSModel();
  gps_sat_thrs = config->getGPSSatThrs();
  gps_hdop_thrs = config->getGPSHDOPThrs();
  const char *pref = config->getPrefix();
  if (*pref != '\0') {
    memcpy(prefix, config->getPrefix(), 4);
  }
  ESP_ERROR_CHECK( esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT) );
  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK( esp_bt_controller_init(&bt_cfg) );
  ESP_ERROR_CHECK( esp_bt_controller_enable(ESP_BT_MODE_BLE) );
  ESP_ERROR_CHECK( esp_bluedroid_init() );
  ESP_ERROR_CHECK( esp_bluedroid_enable() );
  ESP_ERROR_CHECK( esp_ble_gatts_register_callback(gatts_event_handler) );
  ESP_ERROR_CHECK( esp_ble_gap_register_callback(gap_event_handler) );
  ESP_ERROR_CHECK( esp_ble_gatts_app_register(ESP_APP_ID) );
  ESP_ERROR_CHECK( esp_ble_gatt_set_local_mtu(500) );
}
