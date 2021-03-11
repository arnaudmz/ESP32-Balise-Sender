// vim:et:sts=2:sw=2:si
#include "freertos/FreeRTOS.h"
#include "esp_sleep.h"

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

#define uS_TO_mS_FACTOR 1000

Config config;
Switches switches(&config);
TinyGPSPlus gps;
GPSCnx cnx(&gps);
droneIDFR droneID;
LED led;
Beacon beacon(&config, &led, &switches, &droneID, &gps);

unsigned long IRAM_ATTR millis() {
  return (unsigned long) (esp_timer_get_time() / 1000ULL);
}

void low_power(uint32_t delay_ms = 0) {
  ESP_LOGV(TAG, "%ld Sleep", millis());
#if defined(CONFIG_BEACON_GPS_L80R_UART) || defined(CONFIG_BEACON_GPS_L96_UART)
  gpio_wakeup_enable(config.getPPSPort(), GPIO_INTR_HIGH_LEVEL);
  esp_sleep_enable_gpio_wakeup();
  esp_light_sleep_start();
  gpio_wakeup_enable(config.getPPSPort(), GPIO_INTR_LOW_LEVEL);
  esp_light_sleep_start();
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_GPIO);
#endif
#if defined(CONFIG_BEACON_GPS_BN_220_UART) || defined(CONFIG_BEACON_GPS_MOCK)
  esp_sleep_enable_timer_wakeup(delay_ms * uS_TO_mS_FACTOR);
  esp_light_sleep_start();
#endif
  ESP_LOGV(TAG, "%ld Wakeup", millis());
}

void setup() {
  config.begin();
  config.printConfig();
#if defined(CONFIG_BEACON_GPS_L80R_UART) || defined(CONFIG_BEACON_GPS_L96_UART)
  gpio_pad_select_gpio(config.getPPSPort());
  gpio_set_direction(config.getPPSPort(), GPIO_MODE_INPUT);
#endif
  switches.begin();
  cnx.begin();
#ifdef CONFIG_BEACON_GPS_MOCK
  cnx.setBeacon(&beacon);
#endif
  led.begin();
  beacon.begin();
}

void loop() {
  uint32_t first_char_ts, startup_ts = millis();
#if defined(CONFIG_BEACON_GPS_BN_220_UART) || defined(CONFIG_BEACON_GPS_MOCK)
  uint32_t sleep_duration = 990;
#endif
  ESP_LOGV(TAG, "%d Main loop", startup_ts);
  first_char_ts = cnx.waitForChars();
  ESP_LOGD(TAG, "Spent %ldms to read, wasted %d ms", millis() - startup_ts, first_char_ts - startup_ts);
  beacon.handleData();
  if (first_char_ts > 0) {
#if defined(CONFIG_BEACON_GPS_BN_220_UART) || defined(CONFIG_BEACON_GPS_MOCK)
    sleep_duration -= millis() - first_char_ts;
#endif
  }
#if defined(CONFIG_BEACON_GPS_BN_220_UART) || defined(CONFIG_BEACON_GPS_MOCK)
  if (sleep_duration > 0) {
    ESP_LOGV(TAG, "%ld Going to sleep for %dms", millis(), sleep_duration);
    low_power(sleep_duration);
  } else {
    ESP_LOGV(TAG, "%ld Not Going to sleep (%d)ms!", millis(), sleep_duration);
  }
#else
  ESP_LOGV(TAG, "%ld Going to sleep", millis());
  low_power();
#endif
}

extern "C" void app_main(void) {
  setup();
  while(true) {
    loop();
  }
}
