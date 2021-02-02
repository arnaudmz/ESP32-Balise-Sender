// vim:et:sts=2:sw=2:si
#include "freertos/FreeRTOS.h"

#include "esp_event.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_sleep.h"
#include "driver/uart.h"

#include "nvs_flash.h"
#include "string.h"

#include "droneID_FR.h"
#include "TinyGPS++.h"
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
//#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#include "esp_log.h"
#define MOCK_GPS

#define WIFI_CHANNEL CONFIG_WIFI_CHANNEL
#define GPS_RX_IO    (gpio_num_t)CONFIG_GPS_RX_IO
#define GPS_TX_IO    (gpio_num_t)CONFIG_GPS_TX_IO
#define GROUP_MSB_IO (gpio_num_t)CONFIG_GROUP_MSB_IO
#define GROUP_LSB_IO (gpio_num_t)CONFIG_GROUP_LSB_IO
#define MASS_MSB_IO  (gpio_num_t)CONFIG_MASS_MSB_IO
#define MASS_LSB_IO  (gpio_num_t)CONFIG_MASS_LSB_IO
#define PPS_IO       (gpio_num_t)CONFIG_PPS_IO
#define LED_IO       (gpio_num_t)CONFIG_LED_IO

#define GPS_BAUD_RATE 115200
#define mS_TO_S_FACTOR 1000
#define RX_BUF_SIZE 256
//#define PACKET_READ_TICS        (30 / portTICK_RATE_MS)

#define CHARPP(c) (c <= '\r'? 'X': c)

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
uint8_t model_group;
uint8_t model_mass_group;
uint8_t header_size;

TinyGPSPlus gps;
droneIDFR drone_idfr;

char ssid[32];
char mac_str[13];
char drone_id[33];
static const char* TAG = "Main";

esp_err_t event_handler(void *ctx, system_event_t *event) {
  return ESP_OK;
}

void send_beacon(const uint8_t *packet, const uint8_t to_send) {
  ESP_ERROR_CHECK( esp_wifi_start() );
  //ESP_ERROR_CHECK( esp_wifi_set_promiscuous(true) );
  ESP_ERROR_CHECK( esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE) );
  ESP_ERROR_CHECK( esp_wifi_80211_tx(WIFI_IF_STA, packet, to_send, true) );
  ESP_ERROR_CHECK( esp_wifi_stop() );
}

unsigned long IRAM_ATTR millis() {
  return (unsigned long) (esp_timer_get_time() / 1000ULL);
}

void compute_ID() {
  gpio_pulldown_en(GROUP_MSB_IO);
  gpio_pulldown_en(GROUP_LSB_IO);
  gpio_pulldown_en(MASS_MSB_IO);
  gpio_pulldown_en(MASS_LSB_IO);
  model_group =      model_id_to_value[(gpio_get_level(GROUP_MSB_IO) << 1) + gpio_get_level(GROUP_LSB_IO)];
  model_mass_group = mass_id_to_value[(gpio_get_level(MASS_MSB_IO) << 1) + gpio_get_level(MASS_LSB_IO)];
  gpio_pulldown_dis(GROUP_MSB_IO);
  gpio_pulldown_dis(GROUP_LSB_IO);
  gpio_pulldown_dis(MASS_MSB_IO);
  gpio_pulldown_dis(MASS_LSB_IO);
  snprintf(drone_id, 33, "AEM00100000000%1d%03d%12s",
      model_group, model_mass_group, mac_str);
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
    float time_elapsed = (float(millis() - beaconSec) / 1000);
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
  static uint64_t gpsMap = 0;
#ifdef MOCK_GPS
  if (false) {
#else
  if (!gps.location.isValid()) {
#endif
    if (millis() - gpsMap > 1000) {

      ESP_LOGI(TAG, "Positioning(%llu), valid: %d, hdop: %lf", gpsSec++, gps.satellites.value(), gps.hdop.hdop());
      gpio_set_level(LED_IO, 0);
      gpsMap = millis();
      vTaskDelay(10 / portTICK_PERIOD_MS);
      gpio_set_level(LED_IO, 1);
    }
  } else {
#ifdef MOCK_GPS
    if (!has_set_home) {
#else
    if (!has_set_home && gps.satellites.value() > 6 && gps.hdop.hdop() < 2.0) {
#endif
      has_set_home = true;
      home_alt = gps.altitude.meters();
      ESP_LOGI(TAG, "Setting Home Position, Altitude=%d", (int)home_alt);
      drone_idfr.set_home_position(gps.location.lat(), gps.location.lng(), gps.altitude.meters());
      gpio_set_level(LED_IO, 0);
    }
    drone_idfr.set_current_position(gps.location.lat(), gps.location.lng(), gps.altitude.meters());
    drone_idfr.set_heading(gps.course.deg());
    drone_idfr.set_ground_speed(gps.speed.mps());
    drone_idfr.set_heigth(gps.altitude.meters() - home_alt);
    if (millis() - gpsMap > 1000) {
      ESP_LOGI(TAG, "%d:%02d:%02dZ: lng=%.4f, lat=%.4f, satt=%d, hdop=%f",
          gps.time.hour(),
          gps.time.minute(),
          gps.time.second(),
          gps.location.lng(),
          gps.location.lat(),
          gps.satellites.value(),
          gps.hdop.hdop());
      gpsMap = millis();
    }
    compute_and_send_beacon_if_needed();
  }
}

void low_power() {
  ESP_LOGD(TAG, "Sleep");
#ifdef GPS_MODEL_L80_R
  gpio_wakeup_enable(PPS_IO, GPIO_INTR_HIGH_LEVEL);
  esp_sleep_enable_gpio_wakeup();
  esp_light_sleep_start();
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_GPIO);
  gpio_wakeup_enable(PPS_IO, GPIO_INTR_LOW_LEVEL);
  esp_sleep_enable_gpio_wakeup();
  esp_light_sleep_start();
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_GPIO);
#else
  esp_sleep_enable_gpio_wakeup();
  esp_sleep_enable_timer_wakeup(950 * mS_TO_S_FACTOR);
  esp_light_sleep_start();
#endif
  ESP_LOGD(TAG, "Wakeup");
}

void uart_setup() {
  const uart_config_t uart_config = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .rx_flow_ctrl_thresh = 0,
    .source_clk = UART_SCLK_APB,
  };
  // We won't use a buffer for sending data.
  ESP_ERROR_CHECK( uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0) );
  ESP_ERROR_CHECK( uart_param_config(UART_NUM_1, &uart_config) );
  ESP_ERROR_CHECK( uart_set_pin(UART_NUM_1, GPS_TX_IO, GPS_RX_IO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) );
}

void setup() {
  uint8_t mac[6];
  ESP_ERROR_CHECK( esp_read_mac(mac, ESP_MAC_WIFI_STA) );
  snprintf(mac_str, 13, "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  snprintf(ssid, 32, "AEM_%s", mac_str);
  ESP_LOGI(TAG, "SSID: %s", ssid);
  const size_t ssid_size = (sizeof(ssid) / sizeof(*ssid)) - 1; // remove trailling null termination
  beaconPacket[40] = ssid_size;  // set size
  memcpy(&beaconPacket[41], ssid, ssid_size); // set ssid
  memcpy(&beaconPacket[10], mac, 6); // set mac as source
  memcpy(&beaconPacket[16], mac, 6); // set mac as filter
  header_size = 41 + ssid_size;
  gpio_pad_select_gpio(LED_IO);
  gpio_pad_select_gpio(GROUP_MSB_IO);
  gpio_pad_select_gpio(GROUP_LSB_IO);
  gpio_pad_select_gpio(MASS_MSB_IO);
  gpio_pad_select_gpio(MASS_LSB_IO);
  gpio_set_direction(LED_IO, GPIO_MODE_OUTPUT);
  gpio_set_direction(GROUP_MSB_IO, GPIO_MODE_INPUT);
  gpio_set_direction(GROUP_LSB_IO, GPIO_MODE_INPUT);
  gpio_set_direction(MASS_MSB_IO, GPIO_MODE_INPUT);
  gpio_set_direction(MASS_LSB_IO, GPIO_MODE_INPUT);
  nvs_flash_init();
  ESP_ERROR_CHECK( esp_netif_init() );
  ESP_ERROR_CHECK( esp_event_loop_create_default() );
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
  ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
  ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
  uart_setup();
}

void loop() {
  uint8_t c;
  int nb_chars;
  do {
    nb_chars = uart_read_bytes(UART_NUM_1, &c, 1, 100 / portTICK_RATE_MS);
    if(nb_chars > 0) {
      ESP_LOGD(TAG, "%c", c);
      gps.encode(c);
    }
  } while(nb_chars > 0);
  handle_data();
  low_power();
}

extern "C" void app_main(void) {
  setup();
  while(true) {
    loop();
  }
}
