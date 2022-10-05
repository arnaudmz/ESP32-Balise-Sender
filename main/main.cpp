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
#include "driver/gpio.h"
#include "esp_sleep.h"
#include "esp_log.h"

#include "Config.h"
#include "Switches.h"
#include "TinyGPS++.h"
#include "GPSCnx.h"
#include "LED.h"
#include "Beacon.h"
#include "Telemetry.h"
#include "SPort.h"
#include "Jeti.h"
#include "droneID_FR.h"
#if defined(CONFIG_IDF_TARGET_ESP32S2) && defined(CONFIG_USB_CDC_ENABLED)
#include "tinyusb.h"
#include "tusb_cdc_acm.h"
#include "tusb_console.h"
#endif

static constexpr char TAG[] = "Main";

#define uS_TO_mS_FACTOR 1000

unsigned long IRAM_ATTR millis() {
  return (unsigned long) (esp_timer_get_time() / 1000ULL);
}

void low_power(GPSCnx *cnx, uint32_t delay_ms = 0) {
  ESP_LOGV(TAG, "%ld Sleep", millis());
  cnx->lowPower(delay_ms);
  ESP_LOGV(TAG, "%ld Wakeup", millis());
}

void loop(Config *config, GPSCnx * cnx, Beacon *beacon, Telemetry *t) {
  uint32_t first_char_ts, startup_ts = millis();
  ESP_LOGV(TAG, "%d Main loop", startup_ts);
  first_char_ts = cnx->waitForChars();
  ESP_LOGD(TAG, "Spent %ldms to read, wasted %d ms", millis() - startup_ts, first_char_ts - startup_ts);
  beacon->handleData();
  if (config->getTelemetryMode() != TELEMETRY_OFF && t != NULL){
    t->handle(first_char_ts + 950);
  } else {
    uint32_t sleep_duration = 990;
    switch (config->getGPSModel()) {
      case GPS_MODEL_BN_220:
      case GPS_MODEL_AT6558:
      case GPS_MODEL_MOCK:
        if (first_char_ts > 0) {
          sleep_duration -= millis() - first_char_ts;
        }
        if (sleep_duration > 0) {
          ESP_LOGV(TAG, "%ld Going to sleep for %dms", millis(), sleep_duration);
          low_power(cnx, sleep_duration);
        } else {
          ESP_LOGV(TAG, "%ld Not Going to sleep (%d)ms!", millis(), sleep_duration);
        }
        break;
      default:
        ESP_LOGV(TAG, "%ld Going to sleep", millis());
        low_power(cnx);
    }
  }
}

void normal_run(void) {
  Config config;
  config.printConfig();
  Switches switches(&config);
  TinyGPSPlus gps;
  droneIDFR droneID;
  LED led;
  Beacon beacon(&config, &led, &switches, &droneID, &gps);
  GPSCnx *cnx;
  switch (config.getGPSModel()) {
    case GPS_MODEL_MOCK:
      cnx = new GPSMockCnx(&config, &gps, &beacon);
      break;
    case GPS_MODEL_BN_220:
      cnx = new GPSBN220Cnx(&config, &gps);
      break;
    case GPS_MODEL_L80R:
      cnx = new GPSL80RCnx(&config, &gps);
      break;
    case GPS_MODEL_L96_UART:
      cnx = new GPSL96Cnx(&config, &gps);
      break;
    case GPS_MODEL_AT6558:
      cnx = new GPSAT6558Cnx(&config, &gps);
      break;
    default:
      cnx = NULL;
  }
  Telemetry *t = NULL;
  switch(config.getTelemetryMode()) {
    case TELEMETRY_FRSP:
      t = new SPort(&config, &gps, &beacon);
      break;
    case TELEMETRY_JETI:
      t = new JetiTelemetry(&config, &gps, &beacon);
      break;
    case TELEMETRY_JEEX:
      t = new JetiExBusTelemetry(&config, &gps, &beacon);
      break;
    default:
      break;
  }
  ESP_LOGI(TAG, "Entering main loop");
  while(true) {
    loop(&config, cnx, &beacon, t);
  }
}

#ifdef CONFIG_IDF_TARGET_ESP32
void ble_serve(Config *config, LED *led);

void maintenance_run(void) {
  Config config;
  config.printConfig();
  LED led;
  ble_serve(&config, &led);
}
#endif

extern "C" void app_main(void) {
  esp_log_level_set("*", ESP_LOG_ERROR);
  esp_log_level_set("Main", ESP_LOG_VERBOSE);
  esp_log_level_set("LED", ESP_LOG_VERBOSE);
  //esp_log_level_set("Jeti", ESP_LOG_VERBOSE);
  //esp_log_level_set("SPort", ESP_LOG_VERBOSE);
  esp_log_level_set("Config", ESP_LOG_INFO);
  esp_log_level_set("GPSCnx", ESP_LOG_VERBOSE);
  esp_log_level_set("Beacon", ESP_LOG_INFO);
#ifdef CONFIG_IDF_TARGET_ESP32
  bool is_maintenance = (gpio_get_level((gpio_num_t)3) == 0);
  if (is_maintenance) {
    ESP_LOGI(TAG, "Starting in maintenance mode");
    maintenance_run();
  } else {
#else
#if defined(CONFIG_IDF_TARGET_ESP32S2) && defined(CONFIG_USB_CDC_ENABLED)
  tinyusb_config_t tusb_cfg = { 0 }; // the configuration uses default values
  ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
  tinyusb_config_cdcacm_t amc_cfg = { }; // the configuration uses default values
  ESP_ERROR_CHECK(tusb_cdc_acm_init(&amc_cfg));
  esp_tusb_init_console(TINYUSB_CDC_ACM_0); // log to usb
#endif
#endif
    ESP_LOGI(TAG, "Starting in normal mode");
    normal_run();
#ifdef CONFIG_IDF_TARGET_ESP32
  }
#endif
}
