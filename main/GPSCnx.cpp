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
#include "GPSCnx.h"
#include <cstring>
#include <ctype.h>
#include <cmath>
#include "driver/gpio.h"
#include "esp_sleep.h"
#include "esp_log.h"
#if defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32)
#include "ulp_main.h"
#endif

static constexpr char TAG[] = "GPSCnx";

#define GPS_BAUD_RATE 115200
#define PPS_IO       (gpio_num_t)CONFIG_BEACON_GPS_PPS_IO
#ifdef CONFIG_IDF_TARGET_ESP32C3
#define TX_IO 21
#define RX_IO 20
#else
#define TX_IO 32
#define RX_IO 33
#endif
#define uS_TO_mS_FACTOR 1000

uint8_t GPSCnx::computeNMEACksum(const char *st) {
  uint8_t cksum = 0;
  for(int i=0; i < strlen(st); i++) {
    cksum ^= st[i];
  }
  return cksum;
}

uint32_t GPSCnx::computeCASICCksum(uint8_t c, uint8_t id, const uint8_t* payload, uint16_t p_len) {
  uint32_t cksum = (id << 24) | (c << 16) | p_len;
  uint32_t *p = (uint32_t *) payload;
  for (uint16_t i = 0; i < (p_len >> 2); i ++) {                                                               
    cksum += p[i];
  }
  return cksum;
}

void GPSCnx::injectIfNeeded(uint32_t nb_chars, bool inject) {
  if(inject) {
    for(int i = 0; i < nb_chars; i++) {
      gps->encode(rxBuffer[i]);
    }
    ESP_LOGI(TAG, "Injected %d chars", nb_chars);
  } else {
    ESP_LOGV(TAG, "Dropped %d chars", nb_chars);
  }
}

void GPSCnx::lowPower(uint32_t delay_ms) {
#if defined(CONFIG_IDF_TARGET_ESP32)
 //|| defined(CONFIG_IDF_TARGET_ESP32C3)
  esp_sleep_enable_timer_wakeup(delay_ms * uS_TO_mS_FACTOR);
  esp_light_sleep_start();
#else // ESP32S2 and C3 don't sleep, as It will break USB
  vTaskDelay(pdMS_TO_TICKS(delay_ms));
#endif
}

uint32_t GPSUARTCnx::waitForChars(int first_timeout_ms, int next_timeout_ms, bool inject) {
  uint32_t first_char_ts, nb_chars;
  nb_chars = uart_read_bytes(uartPort, rxBuffer, 1, first_timeout_ms / portTICK_PERIOD_MS);
  if (nb_chars == 0) {
    return 0;
  }
  first_char_ts = millis();
  nb_chars += uart_read_bytes(uartPort, &rxBuffer[1], RX_BUF_SIZE - 1, next_timeout_ms / portTICK_PERIOD_MS);
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
  ESP_ERROR_CHECK( uart_wait_tx_done(uartPort, 1000 / portTICK_PERIOD_MS) );
}

void GPSUARTCnx::uartSendCASIC(uint8_t c, uint8_t id, const uint8_t *payload, uint16_t p_len) {
  uint32_t cksum = computeCASICCksum(c, id, payload, p_len);
  const uint8_t prolog[] = {0xba, 0xce, (uint8_t) (p_len & 0xff), (uint8_t) (p_len >> 8), c, id};
  uart_write_bytes(uartPort, prolog, sizeof(prolog));
  if(p_len > 0) {
    uart_write_bytes(uartPort, payload, p_len);
  }
  uart_write_bytes(uartPort, &cksum, sizeof(cksum));
}

GPSBN220Cnx::GPSBN220Cnx(Config *config, TinyGPSPlus *gps): GPSUARTCnx(config, gps) {
  uart_config_t uart_config = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .rx_flow_ctrl_thresh = 0,
    .source_clk = UART_SOURCE,
  };
  // We won't use a buffer for sending data.
  ESP_ERROR_CHECK( uart_driver_install(uartPort, RX_BUF_SIZE * 2, 0 , 0, NULL, 0) );
  ESP_ERROR_CHECK( uart_param_config(uartPort, &uart_config) );
  ESP_ERROR_CHECK( uart_set_pin(uartPort, UART_PIN_NO_CHANGE, RX_IO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) );
}

GPSAT6558Cnx::GPSAT6558Cnx(Config *config, TinyGPSPlus *gps): GPSUARTCnx(config, gps) {
  const char pcas_enable_all_gns[] = "PCAS04,7";
  const char pcas_limit_msg[] = "PCAS03,0,0,0,0,0,0,0,0";
  const char pcas_switch_to_115200[] = "PCAS01,5";
  const char pcas_save_to_flash[] = "PCAS00";
  uart_config_t uart_config = {
    .baud_rate = 9600,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .rx_flow_ctrl_thresh = 0,
    .source_clk = UART_SOURCE,
  };
  // We won't use a buffer for sending data.
  ESP_ERROR_CHECK( uart_driver_install(uartPort, RX_BUF_SIZE * 2, 0 , 0, NULL, 0) );
  ESP_ERROR_CHECK( uart_param_config(uartPort, &uart_config) );
  ESP_ERROR_CHECK( uart_set_pin(uartPort, TX_IO, RX_IO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) );
  vTaskDelay(500 / portTICK_PERIOD_MS);
  uartSendNMEA(pcas_switch_to_115200);
  vTaskDelay(300 / portTICK_PERIOD_MS);
  ESP_ERROR_CHECK( uart_set_baudrate(uartPort, 115200) );
  uartSendNMEA(pcas_limit_msg);
  uartSendNMEA(pcas_enable_all_gns);
  uartSendNMEA(pcas_save_to_flash); // not sure it actually works
}

uint32_t GPSAT6558Cnx::waitForChars(int first_timeout_ms, int next_timeout_ms, bool inject) {
  uint32_t first_char_ts;
  uint32_t query_RMC_msg = 0xffff044e;
  uint32_t query_GGA_msg = 0xffff004e;
  ESP_LOGV(TAG, "Send RMC CMD");
  //uartSendCASIC(0x06, 0x01, (uint8_t *)&query_RMC_msg, 4);
  ESP_LOGV(TAG, "Send GGA CMD");
  //uartSendCASIC(0x06, 0x01, (uint8_t *)&query_GGA_msg, 4);
  uint32_t nb_chars = uart_read_bytes(uartPort, rxBuffer, sizeof(rxBuffer), next_timeout_ms / portTICK_PERIOD_MS);
  if (nb_chars == 0) {
    ESP_LOGW(TAG, "Read nothing");
    return 0;
  }
  first_char_ts = millis();
  const uint8_t *ptr = (const uint8_t *)rxBuffer;
  int16_t r_len = nb_chars;
  while (r_len > 0) {
    if (r_len < 10) {
      ESP_LOGW(TAG, "Error, too few (%d) bytes received", r_len);
      return first_char_ts;
    }
    if (ptr[0] != 0xba || ptr[1] != 0xce) {
      if (ptr[0] != '$' || ptr[1] != 'G') {
        ESP_LOGW(TAG, "Error, invalid message header (0x%02x, 0x%02x) received", ptr[0], ptr[1]);
        return first_char_ts;
      }
      // this looks like NMEA Stuff, look for CRLF
      void *eos = memchr(ptr, 0x0a, r_len);
      if (!eos) {
        ESP_LOGW(TAG, "Error, can't find separator for NMEA String");
        ESP_LOG_BUFFER_HEXDUMP("payload", ptr, r_len, ESP_LOG_ERROR);
        return first_char_ts;
      }
      uint16_t len = (uint32_t)eos - (uint32_t)ptr + 1;
      if (inject) {
        for (int i = 0; i < len; i++) {
          gps->encode(ptr[i]);
        }
        ESP_LOGI(TAG, "Injected %d chars", len);
      }
      ptr += len;
      r_len -= len;
    } else {
      uint16_t p_len = (ptr[3] << 8) | ptr[2];
      if (p_len + 10 > r_len) {
        ESP_LOGW(TAG, "Error, invalid payload length (read 0x%02x, remaning 0x%02x)", p_len, r_len);
        return first_char_ts;
      }
      uint8_t c = ptr[4];
      uint8_t id = ptr[5];
      uint32_t computed_cksum = computeCASICCksum(c, id, &ptr[6], p_len);
      uint32_t *read_cksum = (uint32_t *)&ptr[6 + p_len];
      if (*read_cksum != computed_cksum) {
        ESP_LOGW(TAG, "Invalid msg[%d] cksum: class=0x%02x, id=0x%02x, exp=0x%04x != got=0x%04x", p_len, c, id, computed_cksum, *read_cksum);
        ESP_LOG_BUFFER_HEXDUMP("msg", ptr, p_len + 10, ESP_LOG_ERROR);
      } else {
        // Got a valid response, but do nothing so far with it if it's an ACK-ACK
        if (c != 0x05 || id != 0x01) {
          ESP_LOGW(TAG, "Got valid but unexpected msg: class=0x%02x, id=0x%02x", c, id);
          ESP_LOG_BUFFER_HEXDUMP("payload", &ptr[6], p_len, ESP_LOG_ERROR);
        }
      }
      ptr = &ptr[p_len + 10];
      r_len -= p_len + 10;
    }
  }
  return first_char_ts;
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
  const char pmtk_config_pps[] = "PMTK285,4,160";
  uart_config_t uart_config = {
    .baud_rate = 9600,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .rx_flow_ctrl_thresh = 0,
    .source_clk = UART_SOURCE,
  };
  // We won't use a buffer for sending data.
  ESP_ERROR_CHECK( uart_driver_install(uartPort, RX_BUF_SIZE * 2, 0 , 0, NULL, 0) );
  ESP_ERROR_CHECK( uart_param_config(uartPort, &uart_config) );
  vTaskDelay(500 / portTICK_PERIOD_MS);
  ESP_ERROR_CHECK( uart_set_pin(uartPort, TX_IO, RX_IO, 0, 22) );
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
    .source_clk = UART_SOURCE,
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
  char raw_gprmc_msg[120], raw_gpgga_msg[120];
  int gprmc_cksum, gpgga_cksum;
  if (!beacon->hasSetHomeYet()) {
    lat = mockGPSHomeLat;
    longi = mockGPSHomeLong;
    heading = 0;
    alt = mockGPSHomeAlt + 10;
    hdop -= 0.2;
    if (nb_sat < 7) {
      nb_sat ++;
    }
  } else if(!beacon->hasTakenOffYet()) {
    lat = mockGPSHomeLat;
    longi = mockGPSHomeLong;
    heading = 0;
    if (hdop > 1.3) {
      hdop -= 0.2;
      alt--;
    } else {
      speed_in_knots += 0.5;
    }
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
  snprintf(raw_gpgga_msg, 128, "GPGGA,015606.000,%.4f,N,%.4f,E,1,%d,%.2f,%d.0,M,0.0,M,,",
      lat,
      longi,
      nb_sat,
      hdop,
      alt);
  gpgga_cksum = computeNMEACksum(raw_gpgga_msg);
  snprintf(mock_msg, 256, "$%s*%2X\r\n$%s*%2X\r\n", raw_gprmc_msg, gprmc_cksum, raw_gpgga_msg, gpgga_cksum);
  ESP_LOGV(TAG, "msg: %s", mock_msg);
}
