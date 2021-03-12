// vim:et:sts=2:sw=2:si
#include "freertos/FreeRTOS.h"

#include "Config.h"
#include "Switches.h"
#include "TinyGPS++.h"
#include "GPSCnx.h"
#include "LED.h"
#include "Beacon.h"
#include "droneID_FR.h"

//#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
//#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#define LOG_LOCAL_LEVEL ESP_LOG_INFO
static constexpr char TAG[] = "Main";
#include "esp_log.h"

unsigned long IRAM_ATTR millis() {
  return (unsigned long) (esp_timer_get_time() / 1000ULL);
}

void low_power(GPSCnx *cnx, uint32_t delay_ms = 0) {
  ESP_LOGV(TAG, "%ld Sleep", millis());
  cnx->lowPower(delay_ms);
  ESP_LOGV(TAG, "%ld Wakeup", millis());
}

void loop(Config *config, GPSCnx * cnx, Beacon *beacon) {
  uint32_t first_char_ts, startup_ts = millis();
  uint32_t sleep_duration = 990;
  ESP_LOGV(TAG, "%d Main loop", startup_ts);
  first_char_ts = cnx->waitForChars();
  ESP_LOGD(TAG, "Spent %ldms to read, wasted %d ms", millis() - startup_ts, first_char_ts - startup_ts);
  beacon->handleData();
  switch (config->getGPSModel()) {
    case GPS_MODEL_BN_220:
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

extern "C" void app_main(void) {
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
    default:
      cnx = NULL;
  }
  while(true) {
    loop(&config, cnx, &beacon);
  }
}