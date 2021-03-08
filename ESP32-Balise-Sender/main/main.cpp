// vim:et:sts=2:sw=2:si
#include "freertos/FreeRTOS.h"

#include "esp_event.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_sleep.h"
#include "esp_ota_ops.h"

#include "nvs_flash.h"
#include "string.h"

static const char* TAG = "Main";
//#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
//#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#include "esp_log.h"
#include "TinyGPS++.h"
#include "GPSCnx.h"
#include "droneID_FR.h"

#define WIFI_CHANNEL CONFIG_WIFI_CHANNEL
#if defined(CONFIG_BEACON_GPS_L80R_UART) || defined(CONFIG_BEACON_GPS_L96_UART)
#define PPS_IO       (gpio_num_t)CONFIG_BEACON_GPS_PPS_IO
#endif

#ifdef CONFIG_BEACON_ID_SWITCH
#define GROUP_MSB_IO (gpio_num_t)CONFIG_BEACON_ID_GROUP_MSB_IO
#define GROUP_LSB_IO (gpio_num_t)CONFIG_BEACON_ID_GROUP_LSB_IO
#define MASS_MSB_IO  (gpio_num_t)CONFIG_BEACON_ID_MASS_MSB_IO
#define MASS_LSB_IO  (gpio_num_t)CONFIG_BEACON_ID_MASS_LSB_IO
#else
static_assert(strlen(CONFIG_BEACON_ID_PREFIX) == 4, "CONFIG_BEACON_ID_PREFIX string shoud be 4 char long!");
static_assert(CONFIG_BEACON_ID_PREFIX[4] == 0, "CONFIG_BEACON_ID_PREFIX string shoud be null-terminated!");
#endif

#define LED_IO       (gpio_num_t)CONFIG_BEACON_LED_IO

static_assert(strlen(CONFIG_BEACON_ID_BUILDER) == 3, "BEACON_ID_BUILDER string shoud be 3 char long!");
static_assert(CONFIG_BEACON_ID_BUILDER[3] == 0, "BEACON_ID_BUILDER string shoud be null-terminated!");
static_assert(strlen(CONFIG_BEACON_ID_VERSION) == 3, "BEACON_VERSION string shoud be 3 char long!");
static_assert(CONFIG_BEACON_ID_VERSION[3] == 0, "BEACON_VERSION string shoud be null-terminated!");

#ifdef CONFIG_BEACON_ID_OVERRIDE_MAC
static_assert(strlen(CONFIG_BEACON_ID_OVERRIDE_VALUE) == 12, "CONFIG_BEACON_ID_OVERRIDE_VALUE string shoud be 12 char long!");
static_assert(CONFIG_BEACON_ID_OVERRIDE_VALUE[12] == 0, "CONFIG_BEACON_ID_OVERRIDE_VALUE string shoud be null-terminated!");
#endif

#define uS_TO_mS_FACTOR 1000

static constexpr uint8_t model_id_to_value[] = {1, 2, 3, 4};
static constexpr uint8_t mass_id_to_value[] = {2, 4, 25, 150};

static constexpr uint16_t MAX_BEACON_SIZE = 40 + 32 + droneIDFR::FRAME_PAYLOAD_LEN_MAX;  // default beaconPacket size + max ssid size + max drone id frame size
uint8_t beaconPacket[MAX_BEACON_SIZE] = {
  0x80, 0x00,                                      // 0-1: Frame Control
  0x00, 0x00,                                      // 2-3: Duration
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff,              // 4-9: Destination address (broadcast)
  0x24, 0x62, 0xab, 0xdd, 0xb0, 0xbd,              // 10-15: Source address FAKE  // will be dynamically replaced by real MAC address
  0x24, 0x62, 0xab, 0xdd, 0xb0, 0xbd,              // 16-21: Source address FAKE  // will be dynamically replaced by real MAC address
  0x00, 0x00,                                      // 22-23: Sequence / fragment number (done by the SDK)
  0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,  // 24-31: Timestamp (GETS OVERWRITTEN TO 0 BY HARDWARE)
  0xB8, 0x0B,                                      // 32-33: Beacon interval: set to 3s == 3000TU== BB8, bytes in reverse order  // TODO: manually set it
  0x21, 0x04,                                      // 34-35: Capability info
  0x03, 0x01, 0x06,                                // 36-38: DS Parameter set, current channel 6 (= 0x06), // TODO: manually set it
  0x00, 0x20,                                      // 39-40: SSID parameter set, 0x20:maxlength:content
  // 41-XX: SSID (max 32)
};

bool has_set_home = false;
double home_alt = 0.0;

uint8_t program = 0;
uint64_t gpsSec = 0;

uint64_t beaconSec = 0;
bool stat_led = false;
uint8_t header_size;

TinyGPSPlus gps;
GPSCnx cnx(&gps);
droneIDFR drone_idfr;

char ssid[32];
char id_suffix[13];
char drone_id[33];
uint8_t mac[6];

esp_err_t event_handler(void *ctx, system_event_t *event) {
  return ESP_OK;
}

void send_beacon(const uint8_t *packet, const uint8_t to_send) {
  ESP_ERROR_CHECK( esp_wifi_start() );
  ESP_ERROR_CHECK( esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE) );
  ESP_ERROR_CHECK( esp_wifi_80211_tx(WIFI_IF_STA, packet, to_send, true) );
  ESP_ERROR_CHECK( esp_wifi_stop() );
}

unsigned long IRAM_ATTR millis() {
  return (unsigned long) (esp_timer_get_time() / 1000ULL);
}

void compute_ID() {
#ifdef CONFIG_BEACON_ID_SWITCH
  gpio_pulldown_en(GROUP_MSB_IO);
  gpio_pulldown_en(GROUP_LSB_IO);
  gpio_pulldown_en(MASS_MSB_IO);
  gpio_pulldown_en(MASS_LSB_IO);
  uint8_t model_group =      model_id_to_value[(gpio_get_level(GROUP_MSB_IO) << 1) + gpio_get_level(GROUP_LSB_IO)];
  uint8_t model_mass_group = mass_id_to_value[(gpio_get_level(MASS_MSB_IO) << 1) + gpio_get_level(MASS_LSB_IO)];
  gpio_pulldown_dis(GROUP_MSB_IO);
  gpio_pulldown_dis(GROUP_LSB_IO);
  gpio_pulldown_dis(MASS_MSB_IO);
  gpio_pulldown_dis(MASS_LSB_IO);
  snprintf(drone_id, 33, "%3s%3s00000000%1d%03d%12s",
      CONFIG_BEACON_ID_BUILDER, CONFIG_BEACON_ID_VERSION, model_group, model_mass_group, id_suffix);
#else
  snprintf(drone_id, 33, "%3s%3s00000000%4s%12s",
      CONFIG_BEACON_ID_BUILDER, CONFIG_BEACON_ID_VERSION, CONFIG_BEACON_ID_PREFIX, id_suffix);
#endif
  ESP_LOGD(TAG, "Computed ID: %s", drone_id);
}

void compute_and_send_beacon_if_needed() {
  if (drone_idfr.has_home_set() && drone_idfr.time_to_send()) {
    if (stat_led) {
      gpio_set_level(LED_IO, 1);
      stat_led = false;
    } else {
      gpio_set_level(LED_IO, 0);
      stat_led = true;
    }
    float time_elapsed = (float(millis() - beaconSec) / 1000.0);
    beaconSec = millis();

    ESP_LOGI(TAG, "%.1fs Send beacon: (cause: %s) with %.1fm Speed=%f",
        time_elapsed, drone_idfr.has_pass_distance() ? "Distance" : "Time",
        drone_idfr.get_distance_from_last_position_sent(),
        drone_idfr.get_ground_speed_kmh());
    compute_ID();
    drone_idfr.set_drone_id(drone_id);
    const uint8_t to_send = drone_idfr.generate_beacon_frame(beaconPacket, header_size);
    send_beacon(beaconPacket, to_send);
    drone_idfr.set_last_send();
  }
}

void handle_data() {
  if (!gps.location.isValid()) {
    ESP_LOGI(TAG, "Positioning(%llu), valid: %d, hdop: %lf, time: %02d:%02d:%02d",
        gpsSec++,
        gps.satellites.value(),
        gps.hdop.hdop(),
        gps.time.hour(),
        gps.time.minute(),
        gps.time.second());
    gpio_set_level(LED_IO, 0);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    gpio_set_level(LED_IO, 1);
  } else {
    if (!has_set_home) {
      if (gps.satellites.value() > 6 && gps.hdop.hdop() < 2.0) {
        has_set_home = true;
        home_alt = gps.altitude.meters();
        ESP_LOGI(TAG, "Setting Home Position, Altitude=%d", (int)home_alt);
        drone_idfr.set_home_position(gps.location.lat(), gps.location.lng(), gps.altitude.meters());
        gpio_set_level(LED_IO, 0);
      } else {
        // Looking for better precision to set home, blink twice
        gpio_set_level(LED_IO, 0);
        vTaskDelay(10 / portTICK_PERIOD_MS);
        gpio_set_level(LED_IO, 1);
        vTaskDelay(50 / portTICK_PERIOD_MS);
        gpio_set_level(LED_IO, 0);
        vTaskDelay(10 / portTICK_PERIOD_MS);
        gpio_set_level(LED_IO, 1);
      }
    }
    drone_idfr.set_current_position(gps.location.lat(), gps.location.lng(), gps.altitude.meters());
    drone_idfr.set_heading(gps.course.deg());
    drone_idfr.set_ground_speed(gps.speed.mps());
    drone_idfr.set_heigth(gps.altitude.meters() - home_alt);
    ESP_LOGI(TAG, "%d:%02d:%02dZ: lng=%.4f, lat=%.4f, satt=%d, hdop=%f",
      gps.time.hour(),
      gps.time.minute(),
      gps.time.second(),
      gps.location.lng(),
      gps.location.lat(),
      gps.satellites.value(),
      gps.hdop.hdop());
    compute_and_send_beacon_if_needed();
  }
}

void low_power(uint32_t delay_ms = 0) {
  ESP_LOGV(TAG, "%ld Sleep", millis());
#if defined(CONFIG_BEACON_GPS_L80R_UART) || defined(CONFIG_BEACON_GPS_L96_UART)
  gpio_wakeup_enable(PPS_IO, GPIO_INTR_HIGH_LEVEL);
  esp_sleep_enable_gpio_wakeup();
  esp_light_sleep_start();
  gpio_wakeup_enable(PPS_IO, GPIO_INTR_LOW_LEVEL);
  esp_light_sleep_start();
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_GPIO);
#endif
#if defined(CONFIG_BEACON_GPS_BN_220_UART) || defined(CONFIG_BEACON_GPS_MOCK)
  esp_sleep_enable_gpio_wakeup();
  esp_sleep_enable_timer_wakeup(delay_ms * uS_TO_mS_FACTOR);
  esp_light_sleep_start();
#endif
  ESP_LOGV(TAG, "%ld Wakeup", millis());
}

void print_config() {
  const esp_app_desc_t *app = esp_ota_get_app_description();
  ESP_LOGI(TAG, "Starting Beacon (%s) version %s", app->project_name, app->version);
  ESP_LOGI(TAG, "MAC address: %02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  ESP_LOGI(TAG, "SSID: %s", ssid);
  ESP_LOGI(TAG, "ID Builder: %s", CONFIG_BEACON_ID_BUILDER);
  ESP_LOGI(TAG, "ID Version: %s", CONFIG_BEACON_ID_VERSION);
#ifdef CONFIG_BEACON_ID_OVERRIDE_MAC
  ESP_LOGI(TAG, "ID Suffix (overriden): %s", id_suffix);
#else
  ESP_LOGI(TAG, "ID Suffix (from MAC): %s", id_suffix);
#endif
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
#ifdef CONFIG_BEACON_ID_SWITCH
  ESP_LOGI(TAG, "Swiches are enabled.");
  ESP_LOGI(TAG, "  - IO for Group MSB: %d", GROUP_MSB_IO);
  ESP_LOGI(TAG, "  - IO for Group LSB: %d", GROUP_LSB_IO);
  ESP_LOGI(TAG, "  - IO for Mass MSB: %d", MASS_MSB_IO);
  ESP_LOGI(TAG, "  - IO for Mass LSB: %d", MASS_LSB_IO);
#else
  ESP_LOGI(TAG, "Swiches are disabled.");
  ESP_LOGI(TAG, "Hard coded prefix: %s", CONFIG_BEACON_ID_PREFIX);
#endif
  ESP_LOGI(TAG, "IO for LED: %d", LED_IO);
}

void setup() {
  ESP_ERROR_CHECK( esp_read_mac(mac, ESP_MAC_WIFI_STA) );
#ifdef CONFIG_BEACON_ID_OVERRIDE_MAC
  strncpy(id_suffix, CONFIG_BEACON_ID_OVERRIDE_VALUE, sizeof(id_suffix));
# else
  snprintf(id_suffix, 13, "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
#endif
  snprintf(ssid, 32, "%3s_%s", CONFIG_BEACON_ID_BUILDER, id_suffix);
  const size_t ssid_size = (sizeof(ssid) / sizeof(*ssid)) - 1; // remove trailling null termination
  beaconPacket[40] = ssid_size;  // set size
  memcpy(&beaconPacket[41], ssid, ssid_size); // set ssid
  memcpy(&beaconPacket[10], mac, 6); // set mac as source
  memcpy(&beaconPacket[16], mac, 6); // set mac as filter
  header_size = 41 + ssid_size;
  print_config();
  gpio_pad_select_gpio(LED_IO);
  gpio_set_direction(LED_IO, GPIO_MODE_OUTPUT);
#ifdef CONFIG_BEACON_ID_SWITCH
  gpio_pad_select_gpio(GROUP_MSB_IO);
  gpio_pad_select_gpio(GROUP_LSB_IO);
  gpio_pad_select_gpio(MASS_MSB_IO);
  gpio_pad_select_gpio(MASS_LSB_IO);
  gpio_set_direction(GROUP_MSB_IO, GPIO_MODE_INPUT);
  gpio_set_direction(GROUP_LSB_IO, GPIO_MODE_INPUT);
  gpio_set_direction(MASS_MSB_IO, GPIO_MODE_INPUT);
  gpio_set_direction(MASS_LSB_IO, GPIO_MODE_INPUT);
#endif
#if defined(CONFIG_BEACON_GPS_L80R_UART) || defined(CONFIG_BEACON_GPS_L96_UART)
  gpio_pad_select_gpio(PPS_IO);
  gpio_set_direction(PPS_IO, GPIO_MODE_INPUT);
#endif
  nvs_flash_init();
  ESP_ERROR_CHECK( esp_netif_init() );
  ESP_ERROR_CHECK( esp_event_loop_create_default() );
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
  ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
  ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
  cnx.begin();
}

void loop() {
  uint32_t first_char_ts, startup_ts = millis();
#if defined(CONFIG_BEACON_GPS_BN_220_UART) || defined(CONFIG_BEACON_GPS_MOCK)
  uint32_t sleep_duration = 990;
#endif
  ESP_LOGV(TAG, "%d Main loop", startup_ts);
  first_char_ts = cnx.wait_for_chars();
  ESP_LOGD(TAG, "Spent %ldms to read, wasted %d ms", millis() - startup_ts, first_char_ts - startup_ts);
  handle_data();
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
