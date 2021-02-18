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

static const char* TAG = "Beacon";
#include "TinyGPS++.h"
//#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
//#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#include "esp_log.h"
#include "droneID_FR.h"

#define WIFI_CHANNEL CONFIG_WIFI_CHANNEL

#if defined(CONFIG_BEACON_GPS_L80R) || defined(CONFIG_BEACON_GPS_L96_UART)
#define GPS_RX_IO    (gpio_num_t)CONFIG_BEACON_GPS_RX_IO
#define GPS_TX_IO    (gpio_num_t)CONFIG_BEACON_GPS_TX_IO
#define PPS_IO       (gpio_num_t)CONFIG_BEACON_GPS_PPS_IO
#endif

#ifdef CONFIG_BEACON_GPS_L96_I2C
#define GPS_SDA_IO    (gpio_num_t)CONFIG_BEACON_GPS_SDA_IO
#define GPS_SCL_IO    (gpio_num_t)CONFIG_BEACON_GPS_SCL_IO
#endif

#define GROUP_MSB_IO (gpio_num_t)CONFIG_BEACON_GROUP_MSB_IO
#define GROUP_LSB_IO (gpio_num_t)CONFIG_BEACON_GROUP_LSB_IO
#define MASS_MSB_IO  (gpio_num_t)CONFIG_BEACON_MASS_MSB_IO
#define MASS_LSB_IO  (gpio_num_t)CONFIG_BEACON_MASS_LSB_IO
#define LED_IO       (gpio_num_t)CONFIG_BEACON_LED_IO

static_assert(strlen(CONFIG_BEACON_BUILDER_ID) == 3, "BEACON_VENDOR_ID string shoud be 3 char long!");
static_assert(CONFIG_BEACON_BUILDER_ID[3] == 0, "BEACON_VENDOR_ID string shoud be null-terminated!");
static_assert(strlen(CONFIG_BEACON_VERSION) == 3, "BEACON_VERSION string shoud be 3 char long!");
static_assert(CONFIG_BEACON_VERSION[3] == 0, "BEACON_VERSIOB string shoud be null-terminated!");

#define GPS_BAUD_RATE 115200
#define uS_TO_mS_FACTOR 1000
#define RX_BUF_SIZE 256

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

#ifdef CONFIG_BEACON_GPS_MOCK
char mock_msg[] = "$GPRMC,015606.000,A,3150.7584,N,11712.0491,E,12.30,231.36,280715,,,A*57\r\n"
                  "$GPGGA,015606.000,3150.7584,N,11712.0491,E,1,7,1.28,265.0,M,0.0,M,,*64\r\n";
uint8_t mock_cursor = 0;
#endif

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
  snprintf(drone_id, 33, "%3s%3s00000000%1d%03d%12s",
      CONFIG_BEACON_BUILDER_ID, CONFIG_BEACON_VERSION, model_group, model_mass_group, mac_str);
  ESP_LOGD(TAG, "Computed ID: %s", drone_id);
}

uint8_t st[512];
uint8_t st_i;
bool wait_for_char(int timeout_ms, bool inject) {
  uint8_t c;
  int nb_chars;
#ifdef CONFIG_BEACON_GPS_MOCK
  if (mock_cursor == strlen(mock_msg)) {
    nb_chars = 0;
    mock_cursor = 0;
  } else {
    nb_chars = 1;
    c = mock_msg[mock_cursor++];
  }
#else
  nb_chars = uart_read_bytes(UART_NUM_1, &c, 1, timeout_ms / portTICK_RATE_MS);
#endif
  if(nb_chars > 0) {
    if(inject) {
      gps.encode(c);
      st[st_i++] = c;
    }
    return true;
  }
  return false;
}

void wait_for_silence() {
  while (wait_for_char(30, false)) {
  }
}

void send_PMTK(const char *st) {
  const char prolog='$';
  char crc_buf[6];
  uint8_t cksum = 0;
  for(int i=0; i < strlen(st); i++) {
    cksum ^= st[i];
  }
  snprintf(crc_buf, 6, "*%02X\r\n", cksum);
  wait_for_silence();
  uart_write_bytes(UART_NUM_1, &prolog, 1);
  uart_write_bytes(UART_NUM_1, st, strlen(st));
  uart_write_bytes(UART_NUM_1, (const char*)crc_buf, 5);
  ESP_ERROR_CHECK( uart_wait_tx_done(UART_NUM_1, 1000 / portTICK_RATE_MS) );
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
  if (!gps.location.isValid()) {
    if (millis() - gpsMap > 1000) {
      gpsMap = millis();
      ESP_LOGI(TAG, "Positioning(%llu), valid: %d, hdop: %lf", gpsSec++, gps.satellites.value(), gps.hdop.hdop());
      gpio_set_level(LED_IO, 0);
      vTaskDelay(10 / portTICK_PERIOD_MS);
      gpio_set_level(LED_IO, 1);
    }
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

void low_power(uint32_t delay_ms) {
  ESP_LOGV(TAG, "%ld Sleep", millis());
#ifdef CONFIG_BEACON_GPS_L80R
  gpio_wakeup_enable(PPS_IO, GPIO_INTR_HIGH_LEVEL);
  esp_sleep_enable_gpio_wakeup();
  esp_light_sleep_start();
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_GPIO);
  gpio_wakeup_enable(PPS_IO, GPIO_INTR_LOW_LEVEL);
  esp_sleep_enable_gpio_wakeup();
  esp_light_sleep_start();
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_GPIO);
#endif
#ifdef CONFIG_BEACON_GPS_L96_UART
  gpio_wakeup_enable(PPS_IO, GPIO_INTR_HIGH_LEVEL);
  esp_sleep_enable_gpio_wakeup();
  esp_light_sleep_start();
  ESP_LOGV(TAG, "%ld Wk0", millis());
  gpio_wakeup_enable(PPS_IO, GPIO_INTR_LOW_LEVEL);
  esp_light_sleep_start();
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_GPIO);
#endif
#if defined(CONFIG_BEACON_GPS_BN_220) || defined(CONFIG_BEACON_GPS_MOCK)
  esp_sleep_enable_gpio_wakeup();
  esp_sleep_enable_timer_wakeup(delay_ms * uS_TO_mS_FACTOR);
  esp_light_sleep_start();
#endif
  ESP_LOGV(TAG, "%ld Wakeup", millis());
}

void uart_setup() {
#if defined(CONFIG_BEACON_GPS_L96_UART) || defined(CONFIG_BEACON_GPS_L80R)
  const char pmtk_select_nmea_msg[] = "PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0";
  const char pmtk_config_pps[] = "PMTK285,4,125";
#ifdef CONFIG_BEACON_GPS_L80R
  const char pmtk_switch_baud_rate[] = "PMTK251,115200";
  const char pmtk_enable_pps[] = "PMTK255,1";
#endif //ifdef CONFIG_BEACON_GPS_L80R
#endif
  const uart_config_t uart_config = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .rx_flow_ctrl_thresh = 0,
    .source_clk = UART_SCLK_REF_TICK,
  };
  // We won't use a buffer for sending data.
  ESP_ERROR_CHECK( uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0 , 0, NULL, 0) );
#ifdef CONFIG_BEACON_GPS_L80R
  uart_config.bad_rate = 9600;
#endif
  ESP_ERROR_CHECK( uart_param_config(UART_NUM_1, &uart_config) );
#ifdef CONFIG_BEACON_GPS_BN_220
  ESP_ERROR_CHECK( uart_set_pin(UART_NUM_1, UART_PIN_NO_CHANGE, 16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) );
#endif
#ifdef CONFIG_BEACON_GPS_L96_UART
  ESP_ERROR_CHECK( uart_set_pin(UART_NUM_1, GPS_TX_IO, GPS_RX_IO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) );
  send_PMTK(pmtk_select_nmea_msg);
  send_PMTK(pmtk_config_pps);
  wait_for_silence();
#endif
#ifdef CONFIG_BEACON_GPS_L80R
  ESP_ERROR_CHECK( uart_set_pin(UART_NUM_1, GPS_TX_IO, GPS_RX_IO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) );
  send_PMTK(pmtk_select_nmea_msg);
  ESP_ERROR_CHECK( uart_set_baudrate(UART_NUM_1, 115200) );
  send_PMTK(pmtk_enable_pps);
  send_PMTK(pmtk_config_pps);
  wait_for_silence();
#endif
}

#ifdef CONFIG_BEACON_GPS_L96_I2C
void i2c_setup() {
  static i2c_port_t i2c_port  = I2C_NUM_0;
  static uint32_t i2c_frequency = 400000;
  i2c_config_t conf = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = GPS_SDA_IO,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_io_num = GPS_SCL_IO,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = i2c_frequency
  };
  i2c_driver_install(i2c_port, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
  ESP_ERROR_CHECK( i2c_param_config(i2c_port, &conf) );
}
#endif

void gps_setup() {
#if defined(CONFIG_BEACON_GPS_MOCK) || defined(CONFIG_BEACON_GPS_L96_I2C)
#ifdef CONFIG_BEACON_GPS_L96_I2C
  i2c_setup();
#endif // ifdef CONFIG_BEACON_GPS_L96_I2C
#else
  uart_setup();
#endif
}

void setup() {
  uint8_t mac[6];
  ESP_ERROR_CHECK( esp_read_mac(mac, ESP_MAC_WIFI_STA) );
  snprintf(mac_str, 13, "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  snprintf(ssid, 32, "%3s_%s", CONFIG_BEACON_BUILDER_ID, mac_str);
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
#if defined(CONFIG_BEACON_GPS_L80R) || defined(CONFIG_BEACON_GPS_L96_UART)
  gpio_pad_select_gpio(PPS_IO);
#endif
  gpio_set_direction(LED_IO, GPIO_MODE_OUTPUT);
  gpio_set_direction(GROUP_MSB_IO, GPIO_MODE_INPUT);
  gpio_set_direction(GROUP_LSB_IO, GPIO_MODE_INPUT);
  gpio_set_direction(MASS_MSB_IO, GPIO_MODE_INPUT);
  gpio_set_direction(MASS_LSB_IO, GPIO_MODE_INPUT);
#if defined(CONFIG_BEACON_GPS_L80R) || defined(CONFIG_BEACON_GPS_L96_UART)
  gpio_set_direction(PPS_IO, GPIO_MODE_INPUT);
#endif
  nvs_flash_init();
  ESP_ERROR_CHECK( esp_netif_init() );
  ESP_ERROR_CHECK( esp_event_loop_create_default() );
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
  ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
  ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
  gps_setup();
}

void loop() {
  uint32_t startup_ts = millis();
  uint32_t first_char_ts = 0;
  uint32_t last_char_ts = 0;
#if defined(CONFIG_BEACON_GPS_BN_220) || defined(CONFIG_BEACON_GPS_MOCK)
  uint32_t sleep_duration = 990;
#endif
  ESP_LOGV(TAG, "%d Wakeup", startup_ts);
  st_i=0;
  if (wait_for_char(1000, true)) {
    first_char_ts = millis();
    ESP_LOGV(TAG, "%d First Char, wasted %dms", first_char_ts, first_char_ts - startup_ts);
    while (wait_for_char(20, true)) {
    }
    ESP_LOGV(TAG, "%ld Last Char", millis());
    last_char_ts = millis();
    ESP_LOGD(TAG, "%d lost %dms then read %d chars in %d ms", last_char_ts, first_char_ts - startup_ts, st_i, last_char_ts - first_char_ts);
  }
  handle_data();
  if (first_char_ts > 0) {
    ESP_LOG_BUFFER_HEXDUMP(TAG, st, st_i, ESP_LOG_VERBOSE);
#if defined(CONFIG_BEACON_GPS_BN_220) || defined(CONFIG_BEACON_GPS_MOCK)
    sleep_duration -= millis() - first_char_ts;
#endif
  }
#if defined(CONFIG_BEACON_GPS_BN_220) || defined(CONFIG_BEACON_GPS_MOCK)
  ESP_LOGV(TAG, "%ld Going to sleep for %dms", millis(), sleep_duration);
  low_power(sleep_duration);
#else
  ESP_LOGV(TAG, "%ld Going to sleep", millis());
  low_power(0);
#endif
}
int last=0;
extern "C" void app_main(void) {
  setup();
  //gpio_pulldown_en(PPS_IO);
  while(true) {
    loop();
    /*int val = gpio_get_level(PPS_IO);
    if (val != last) {
      last = val;
      ESP_LOGD(TAG, "%ld Value: %d", millis(), val);
    }
    vTaskDelay(1);*/
  }
}
