// vim:et:sts=2:sw=2:si
#include "GPSCnx.h"
#include <cstring>
#include <ctype.h>
#include <cmath>
//#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
//#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#define LOG_LOCAL_LEVEL ESP_LOG_INFO
static constexpr char TAG[] = "GPSCnx";
#include "esp_log.h"

#if defined(CONFIG_BEACON_GPS_L80R_UART) || defined(CONFIG_BEACON_GPS_L96_UART) || defined(CONFIG_BEACON_GPS_BN_220_UART)
#define GPS_BAUD_RATE 115200
#if defined(CONFIG_BEACON_GPS_L80R_UART) || defined(CONFIG_BEACON_GPS_L96_UART)
#define PPS_IO       (gpio_num_t)CONFIG_BEACON_GPS_PPS_IO
#define TX_IO 17
#define RX_IO 16
#endif //#if defined(CONFIG_BEACON_GPS_L80R_UART) || defined(CONFIG_BEACON_GPS_L96_UART)

#ifdef CONFIG_BEACON_GPS_BN_220_UART
#define RX_IO 16
#endif //#ifdef CONFIG_BEACON_GPS_BN_220_UART
#endif //#if defined(CONFIG_BEACON_GPS_L80R_UART) || defined(CONFIG_BEACON_GPS_L96_UART) || defined(CONFIG_BEACON_GPS_BN_220_UART)

uint8_t GPSCnx::computePMTKCksum(const char *st) {
  uint8_t cksum = 0;
  for(int i=0; i < strlen(st); i++) {
    cksum ^= st[i];
  }
  return cksum;
}

#ifdef CONFIG_BEACON_GPS_L96_I2C

#define GPS_SDA_IO    (gpio_num_t)CONFIG_BEACON_GPS_SDA_IO
#define GPS_SCL_IO    (gpio_num_t)CONFIG_BEACON_GPS_SCL_IO

#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0
#define I2C_WR_ADDRESS 0x20
#define I2C_RD_ADDRESS 0x21
#define I2C_BUF_SIZE 255
#define ACK_CHECK_EN 0x1
#define ACK_CHECK_DIS 0x0
#define ACK_VAL 0x0
#define NACK_VAL 0x1
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0

void GPSCnx::i2cRead() {
  ESP_ERROR_CHECK( i2c_driver_install(i2c_port, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0) );
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, I2C_WR_ADDRESS, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, 0, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_RATE_MS);
  ESP_LOGV(TAG, "Wr ret: 0x%x", ret);
  i2c_cmd_link_delete(cmd);
  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, I2C_RD_ADDRESS, ACK_CHECK_EN);
  i2c_master_read(cmd, i2c_buf, I2C_BUF_SIZE - 1, I2C_MASTER_ACK);
  i2c_master_read_byte(cmd, &i2c_buf[I2C_BUF_SIZE - 1], I2C_MASTER_NACK);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_RATE_MS);
  ESP_LOGV(TAG, "Wr ret: 0x%x", ret);
  i2c_cmd_link_delete(cmd);
  ESP_LOG_BUFFER_HEXDUMP(TAG, i2c_buf, I2C_BUF_SIZE, ESP_LOG_VERBOSE);
  ESP_ERROR_CHECK( i2c_driver_delete(i2c_port) );
}
#endif //#ifdef CONFIG_BEACON_GPS_L96_I2C

uint32_t GPSCnx::waitForChars(int first_timeout_ms, int next_timeout_ms, bool inject) {
  uint32_t first_char_ts, nb_chars;
#ifdef CONFIG_BEACON_GPS_MOCK
  computeMockMsg();
  nb_chars = strlen(mock_msg);
  memcpy(rxBuffer, mock_msg, nb_chars);
  first_char_ts = millis();
#else
#ifdef CONFIG_BEACON_GPS_L96_I2C
  i2cRead();
  first_char_ts = millis();
  nb_chars = 0;
#else
  nb_chars = uart_read_bytes(uartPort, rxBuffer, 1, first_timeout_ms / portTICK_RATE_MS);
  if (nb_chars == 0) {
    return 0;
  }
  first_char_ts = millis();
  nb_chars += uart_read_bytes(uartPort, &rxBuffer[1], RX_BUF_SIZE, next_timeout_ms / portTICK_RATE_MS);
#endif //#ifdef CONFIG_BEACON_GPS_L96_I2C
#endif //#ifdef CONFIG_BEACON_GPS_MOCK
  if(inject) {
    for(int i = 0; i < nb_chars; i++) {
      gps->encode(rxBuffer[i]);
    }
    ESP_LOGV(TAG, "Injected %d chars", nb_chars);
  } else {
    ESP_LOGV(TAG, "Dropped %d chars", nb_chars);
  }
  ESP_LOG_BUFFER_HEXDUMP(TAG, rxBuffer, nb_chars, ESP_LOG_VERBOSE);
  return first_char_ts;
}

#if defined(CONFIG_BEACON_GPS_L96_UART) || defined(CONFIG_BEACON_GPS_L80R_UART) || defined(CONFIG_BEACON_GPS_BN_220_UART)
void GPSCnx::uartWaitForSilence(int timeout_ms) {
  uint32_t start = millis();
  ESP_ERROR_CHECK( uart_flush_input(uartPort)  );
  waitForChars(timeout_ms, timeout_ms, false);
  ESP_LOGV(TAG, "Awaited %ld ms", millis() - start);
}

void GPSCnx::uartSendPMTK(const char *st) {
  const char prolog='$';
  char crc_buf[6];
  uint8_t cksum = computePMTKCksum(st);
  snprintf(crc_buf, 6, "*%02X\r\n", cksum);
  uartWaitForSilence();
  uart_write_bytes(uartPort, &prolog, 1);
  uart_write_bytes(uartPort, st, strlen(st));
  uart_write_bytes(uartPort, (const char*)crc_buf, 5);
  ESP_ERROR_CHECK( uart_wait_tx_done(uartPort, 1000 / portTICK_RATE_MS) );
}

void GPSCnx::uartSetup() {
#if defined(CONFIG_BEACON_GPS_L96_UART) || defined(CONFIG_BEACON_GPS_L80R_UART)
  const char pmtk_select_nmea_msg[] = "PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0";
#endif //#if defined(CONFIG_BEACON_GPS_L96_UART) || defined(CONFIG_BEACON_GPS_L80R_UART)
#ifdef CONFIG_BEACON_GPS_L96_UART
  const char pmtk_config_pps[] = "PMTK285,4,125";
#endif //ifdef CONFIG_BEACON_GPS_L96_UART
#ifdef CONFIG_BEACON_GPS_L80R_UART
  const char pmtk_switch_baud_rate[] = "PMTK251,115200";
  const char pmtk_enable_pps[] = "PMTK255,1";
  const char pmtk_config_pps[] = "PMTK285,4,175";
#endif //ifdef CONFIG_BEACON_GPS_L80R_UART
  const uart_config_t uart_config = {
#ifdef CONFIG_BEACON_GPS_L80R_UART
    .baud_rate = 9600,
#else //#ifdef CONFIG_BEACON_GPS_L80R_UART
    .baud_rate = 115200,
#endif //#ifdef CONFIG_BEACON_GPS_L80R_UART
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
#ifdef CONFIG_BEACON_GPS_BN_220_UART
  ESP_ERROR_CHECK( uart_set_pin(uartPort, UART_PIN_NO_CHANGE, RX_IO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) );
#endif //#ifdef CONFIG_BEACON_GPS_BN_220_UART
#ifdef CONFIG_BEACON_GPS_L96_UART
  vTaskDelay(1500 / portTICK_PERIOD_MS);
  ESP_ERROR_CHECK( uart_set_pin(uartPort, TX_IO, RX_IO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) );
  uartWaitForSilence(100);
  uartSendPMTK(pmtk_select_nmea_msg);
  uartSendPMTK(pmtk_config_pps);
  uartWaitForSilence();
#endif //#ifdef CONFIG_BEACON_GPS_L96_UART
#ifdef CONFIG_BEACON_GPS_L80R_UART
  vTaskDelay(500 / portTICK_PERIOD_MS);
  ESP_ERROR_CHECK( uart_set_pin(uartPort, TX_IO, RX_IO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) );
  uartSendPMTK(pmtk_switch_baud_rate);
  vTaskDelay(300 / portTICK_PERIOD_MS);
  ESP_ERROR_CHECK( uart_set_baudrate(uartPort, 115200) );
  uartSendPMTK(pmtk_select_nmea_msg);
  uartSendPMTK(pmtk_config_pps);
  uartSendPMTK(pmtk_enable_pps);
  uartWaitForSilence();
#endif //#ifdef CONFIG_BEACON_GPS_L80R_UART
}
#endif //#if defined(CONFIG_BEACON_GPS_L96_UART) || defined(CONFIG_BEACON_GPS_L80R_UART) || defined(CONFIG_BEACON_GPS_BN_220_UART)

#ifdef CONFIG_BEACON_GPS_L96_I2C

void GPSCnx::i2cSetup() {
  vTaskDelay(500 / portTICK_PERIOD_MS);
  static uint32_t i2c_frequency = 400000;
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = GPS_SDA_IO;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_io_num = GPS_SCL_IO;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = i2c_frequency;
  ESP_ERROR_CHECK( i2c_param_config(i2c_port, &conf) );
  i2cRead();
}
#endif //#ifdef CONFIG_BEACON_GPS_L96_I2C

void GPSCnx::begin() {
#if defined(CONFIG_BEACON_GPS_MOCK) || defined(CONFIG_BEACON_GPS_L96_I2C)
#ifdef CONFIG_BEACON_GPS_L96_I2C
  i2cSetup();
#endif // ifdef CONFIG_BEACON_GPS_L96_I2C
#else
  uartSetup();
#endif //#if defined(CONFIG_BEACON_GPS_MOCK) || defined(CONFIG_BEACON_GPS_L96_I2C)
}

#ifdef CONFIG_BEACON_GPS_MOCK
void GPSCnx::setBeacon(Beacon *b) {
  beacon = b;
}

void GPSCnx::computeMockMsg() {
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
  gprmc_cksum = computePMTKCksum(raw_gprmc_msg);
  snprintf(raw_gpgga_msg, 128, "GPGGA,015606.000,%.4f,N,%.4f,E,1,7,1.28,%d.0,M,0.0,M,,",
      lat,
      longi,
      alt);
  gpgga_cksum = computePMTKCksum(raw_gpgga_msg);
  snprintf(mock_msg, 256, "$%s*%2X\r\n$%s*%2X\r\n", raw_gprmc_msg, gprmc_cksum, raw_gpgga_msg, gpgga_cksum);
  ESP_LOGV(TAG, "msg: %s", mock_msg);
}
#endif
