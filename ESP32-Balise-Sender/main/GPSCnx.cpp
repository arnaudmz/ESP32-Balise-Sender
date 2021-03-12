// vim:et:sts=2:sw=2:si
#include "GPSCnx.h"
#include <cstring>
#include <ctype.h>
#include <cmath>
#include "driver/gpio.h"
#include "esp_sleep.h"

//#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
//#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#define LOG_LOCAL_LEVEL ESP_LOG_INFO
static constexpr char TAG[] = "GPSCnx";
#include "esp_log.h"

#define GPS_BAUD_RATE 115200
#define PPS_IO       (gpio_num_t)CONFIG_BEACON_GPS_PPS_IO
#define TX_IO 17
#define RX_IO 16
#define uS_TO_mS_FACTOR 1000

uint8_t GPSCnx::computeNMEACksum(const char *st) {
  uint8_t cksum = 0;
  for(int i=0; i < strlen(st); i++) {
    cksum ^= st[i];
  }
  return cksum;
}

void GPSCnx::injectIfNeeded(uint32_t nb_chars, bool inject) {
  if(inject) {
    for(int i = 0; i < nb_chars; i++) {
      gps->encode(rxBuffer[i]);
    }
    ESP_LOGV(TAG, "Injected %d chars", nb_chars);
  } else {
    ESP_LOGV(TAG, "Dropped %d chars", nb_chars);
  }
}

void GPSCnx::lowPower(uint32_t delay_ms) {
  esp_sleep_enable_timer_wakeup(delay_ms * uS_TO_mS_FACTOR);
  esp_light_sleep_start();
}

uint32_t GPSUARTCnx::waitForChars(int first_timeout_ms, int next_timeout_ms, bool inject) {
  uint32_t first_char_ts, nb_chars;
  nb_chars = uart_read_bytes(uartPort, rxBuffer, 1, first_timeout_ms / portTICK_RATE_MS);
  if (nb_chars == 0) {
    return 0;
  }
  first_char_ts = millis();
  nb_chars += uart_read_bytes(uartPort, &rxBuffer[1], RX_BUF_SIZE, next_timeout_ms / portTICK_RATE_MS);
  injectIfNeeded(nb_chars, inject);
  ESP_LOG_BUFFER_HEXDUMP(TAG, rxBuffer, nb_chars, ESP_LOG_VERBOSE);
  return first_char_ts;
}

void GPSUARTCnx::uartWaitForSilence(int timeout_ms) {
  uint32_t start = millis();
  ESP_ERROR_CHECK( uart_flush_input(uartPort)  );
  waitForChars(timeout_ms, timeout_ms, false);
  ESP_LOGV(TAG, "Awaited %ld ms", millis() - start);
}

void GPSUARTCnx::uartSendNMEA(const char *st) {
  const char prolog='$';
  char crc_buf[6];
  uint8_t cksum = computeNMEACksum(st);
  snprintf(crc_buf, 6, "*%02X\r\n", cksum);
  uartWaitForSilence();
  uart_write_bytes(uartPort, &prolog, 1);
  uart_write_bytes(uartPort, st, strlen(st));
  uart_write_bytes(uartPort, (const char*)crc_buf, 5);
  ESP_ERROR_CHECK( uart_wait_tx_done(uartPort, 1000 / portTICK_RATE_MS) );
}

GPSBN220Cnx::GPSBN220Cnx(Config *config, TinyGPSPlus *gps): GPSUARTCnx(config, gps) {
  uart_config_t uart_config = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .rx_flow_ctrl_thresh = 0,
    .source_clk = UART_SCLK_REF_TICK,
  };
  // We won't use a buffer for sending data.
  ESP_ERROR_CHECK( uart_driver_install(uartPort, RX_BUF_SIZE * 2, 0 , 0, NULL, 0) );
  ESP_ERROR_CHECK( uart_param_config(uartPort, &uart_config) );
  ESP_ERROR_CHECK( uart_set_pin(uartPort, UART_PIN_NO_CHANGE, RX_IO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) );
}

GPSPPSUARTCnx::GPSPPSUARTCnx(Config *config, TinyGPSPlus *gps): GPSUARTCnx(config, gps) {
  gpio_pad_select_gpio(config->getPPSPort());
  gpio_set_direction(config->getPPSPort(), GPIO_MODE_INPUT);
}

void GPSPPSUARTCnx::lowPower(uint32_t delay_ms) {
  gpio_wakeup_enable(config->getPPSPort(), GPIO_INTR_HIGH_LEVEL);
  esp_sleep_enable_gpio_wakeup();
  esp_light_sleep_start();
  gpio_wakeup_enable(config->getPPSPort(), GPIO_INTR_LOW_LEVEL);
  esp_light_sleep_start();
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_GPIO);
}

GPSL80RCnx::GPSL80RCnx(Config *config, TinyGPSPlus *gps): GPSPPSUARTCnx(config, gps) {
  const char pmtk_select_nmea_msg[] = "PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0";
  const char pmtk_switch_baud_rate[] = "PMTK251,115200";
  const char pmtk_enable_pps[] = "PMTK255,1";
  const char pmtk_config_pps[] = "PMTK285,4,175";
  uart_config_t uart_config = {
    .baud_rate = 9600,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .rx_flow_ctrl_thresh = 0,
    .source_clk = UART_SCLK_REF_TICK,
  };
  // We won't use a buffer for sending data.
  ESP_ERROR_CHECK( uart_driver_install(uartPort, RX_BUF_SIZE * 2, 0 , 0, NULL, 0) );
  ESP_ERROR_CHECK( uart_param_config(uartPort, &uart_config) );
  vTaskDelay(500 / portTICK_PERIOD_MS);
  ESP_ERROR_CHECK( uart_set_pin(uartPort, TX_IO, RX_IO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) );
  uartSendNMEA(pmtk_switch_baud_rate);
  vTaskDelay(300 / portTICK_PERIOD_MS);
  ESP_ERROR_CHECK( uart_set_baudrate(uartPort, 115200) );
  uartSendNMEA(pmtk_select_nmea_msg);
  uartSendNMEA(pmtk_config_pps);
  uartSendNMEA(pmtk_enable_pps);
  uartWaitForSilence();
}

GPSL96Cnx::GPSL96Cnx(Config *config, TinyGPSPlus *gps): GPSPPSUARTCnx(config, gps) {
  const char pmtk_select_nmea_msg[] = "PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0";
  const char pmtk_config_pps[] = "PMTK285,4,125";
  uart_config_t uart_config = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .rx_flow_ctrl_thresh = 0,
    .source_clk = UART_SCLK_REF_TICK,
  };
  // We won't use a buffer for sending data.
  ESP_ERROR_CHECK( uart_driver_install(uartPort, RX_BUF_SIZE * 2, 0 , 0, NULL, 0) );
  ESP_ERROR_CHECK( uart_param_config(uartPort, &uart_config) );
  vTaskDelay(1500 / portTICK_PERIOD_MS);
  ESP_ERROR_CHECK( uart_set_pin(uartPort, TX_IO, RX_IO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) );
  uartWaitForSilence(100);
  uartSendNMEA(pmtk_select_nmea_msg);
  uartSendNMEA(pmtk_config_pps);
  uartWaitForSilence();
}

uint32_t GPSMockCnx::waitForChars(int first_timeout_ms, int next_timeout_ms, bool inject) {
  uint32_t first_char_ts, nb_chars;
  computeMockMsg();
  nb_chars = strlen(mock_msg);
  memcpy(rxBuffer, mock_msg, nb_chars);
  first_char_ts = millis();
  injectIfNeeded(nb_chars, inject);
  return first_char_ts;
}

void GPSMockCnx::computeMockMsg() {
  double lat;
  double longi;
  double heading;
  double speed_in_knots;
  int alt;
  char raw_gprmc_msg[120], raw_gpgga_msg[120];
  int gprmc_cksum, gpgga_cksum;
  if (!beacon->hasSetHomeYet()) {
    lat = mockGPSHomeLat;
    longi = mockGPSHomeLong;
    heading = 0;
    speed_in_knots = 0.0;
    alt = mockGPSHomeAlt;
  } else {
    int secs_from_start = millis() / 1000;
    double pos_angle = (double)secs_from_start * M_TWOPI / 66.33;
    double alt_angle = (double)secs_from_start * M_TWOPI / 92.35;
    ESP_LOGV(TAG, "Computed pos_angle: %f", pos_angle);
    ESP_LOGV(TAG, "Computed alt_angle: %f", alt_angle);
    lat = mockGPSHomeLat + mockGPSRaduis * sin(pos_angle);
    longi = mockGPSHomeLong + mockGPSRaduis * cos(pos_angle);
    heading = fmod(360 - fmod(pos_angle * 360.0 / M_TWOPI, 360), 360);
    speed_in_knots = 20.0 + 10.0 * sin(pos_angle);
    alt = mockGPSHomeAlt + (mockGPSMaxHeight + mockGPSMinHeight) / 2 + (mockGPSMaxHeight -  mockGPSMinHeight) / 2 * cos(alt_angle);
  }
  ESP_LOGV(TAG, "Computed heading: %f", heading);
  ESP_LOGV(TAG, "Computed speed: %f", speed_in_knots);
  ESP_LOGV(TAG, "Computed alt: %d", alt);
  ESP_LOGV(TAG, "Computed lat: %f", lat);
  ESP_LOGV(TAG, "Computed longi: %f", longi);
  snprintf(raw_gprmc_msg, 128, "GPRMC,015606.000,A,%.4f,N,%4f,E,%.2f,%.2f,280715,,,A",
      lat,
      longi,
      speed_in_knots,
      heading);
  gprmc_cksum = computeNMEACksum(raw_gprmc_msg);
  snprintf(raw_gpgga_msg, 128, "GPGGA,015606.000,%.4f,N,%.4f,E,1,7,1.28,%d.0,M,0.0,M,,",
      lat,
      longi,
      alt);
  gpgga_cksum = computeNMEACksum(raw_gpgga_msg);
  snprintf(mock_msg, 256, "$%s*%2X\r\n$%s*%2X\r\n", raw_gprmc_msg, gprmc_cksum, raw_gpgga_msg, gpgga_cksum);
  ESP_LOGV(TAG, "msg: %s", mock_msg);
}
