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
#include "SPort.h"
#include <cstring>
#include <ctype.h>
#include <cmath>
#include "driver/gpio.h"
#include "esp_sleep.h"

//#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
//#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#define LOG_LOCAL_LEVEL ESP_LOG_INFO
static constexpr char TAG[] = "SPort";
#include "esp_log.h"

#define SPORT_BAUD_RATE 57600
#define RX_IO 5
#define TX_IO 18
#define uS_TO_mS_FACTOR 1000

SPortMetric gpsMetrics[SPORT_GPS_METRIC_LAST] = {
    SPortMetric(SPORT_GPS_METRIC_LAT,       SPORT_GPS_METRIC_LAT_LON_ID,   1000),
    SPortMetric(SPORT_GPS_METRIC_LON,       SPORT_GPS_METRIC_LAT_LON_ID,   1000),
    SPortMetric(SPORT_GPS_METRIC_ALT,       SPORT_GPS_METRIC_ALT_ID,       1000),
    SPortMetric(SPORT_GPS_METRIC_SPEED,     SPORT_GPS_METRIC_SPEED_ID,     1000),
    SPortMetric(SPORT_GPS_METRIC_COG,       SPORT_GPS_METRIC_COG_ID,       1000),
    SPortMetric(SPORT_GPS_METRIC_DATE,      SPORT_GPS_METRIC_TS_ID,        10000),
    SPortMetric(SPORT_GPS_METRIC_TIME,      SPORT_GPS_METRIC_TS_ID,        1000),
    SPortMetric(SPORT_BEACON_METRIC_STAT,   SPORT_BEACON_METRIC_STAT_ID,   1000),
    SPortMetric(SPORT_BEACON_METRIC_SAT,    SPORT_BEACON_METRIC_SAT_ID,    1000),
    SPortMetric(SPORT_BEACON_METRIC_HDOP,   SPORT_BEACON_METRIC_HDOP_ID,   1000),
    SPortMetric(SPORT_BEACON_METRIC_PREFIX, SPORT_BEACON_METRIC_PREFIX_ID, 1000),
    SPortMetric(SPORT_BEACON_METRIC_FRAMES, SPORT_BEACON_METRIC_FRAMES_ID, 5000)
};

SPort::SPort(Config *c, TinyGPSPlus *gps, Beacon *beacon):
config(c),
gps(gps),
beacon(beacon) {
  uart_config_t uart_config = {
    .baud_rate = SPORT_BAUD_RATE,
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
  ESP_ERROR_CHECK( uart_set_pin(uartPort, TX_IO, RX_IO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) );
  ESP_ERROR_CHECK( uart_set_line_inverse(uartPort, UART_SIGNAL_RXD_INV | UART_SIGNAL_TXD_INV) );
  ESP_ERROR_CHECK( gpio_pullup_dis((gpio_num_t) RX_IO) );
  gpio_pad_select_gpio(GPIO_NUM_17);
  ESP_ERROR_CHECK( gpio_set_direction(GPIO_NUM_17, GPIO_MODE_OUTPUT) );
  gpio_set_level(GPIO_NUM_17, 0);
}

void SPort::sendChar(char c) {
  uart_write_bytes(uartPort, &c, 1);
}

void SPort::sendEncodedChar(char c) {
  if(c == 0x7e) {
    sendChar(0x7d);
    sendChar(0x5e);
  } else if(c == 0x7d) {
    sendChar(0x7d);
    sendChar(0x5d);
  } else {
    sendChar(c);
  }
}

char SPort::cksum(const char data[], int start, int len) {
  long total = 0;

  for(int i = start; i < (start + len); i++) {
    total += data[i];
  }

  if(total >= 0x700) {
    total+= 7;
  } else if(total >= 0x600) {
    total+= 6;
  } else if(total >= 0x500) {
    total+= 5;
  } else if(total >= 0x400) {
    total+= 4;
  } else if(total >= 0x300) {
    total+= 3;
  } else if(total >= 0x200) {
    total+= 2;
  } else if(total >= 0x100) {
    total++;
  }

  return 0xff - total;
}

void SPort::sendTooSoon() {
  char too_soon[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff};
  ESP_ERROR_CHECK( uart_set_pin(uartPort, RX_IO, TX_IO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) );
  ESP_ERROR_CHECK( gpio_pullup_dis((gpio_num_t) RX_IO) );
  uart_write_bytes(uartPort, too_soon, 8);
  ESP_ERROR_CHECK( uart_wait_tx_done(uartPort, pdMS_TO_TICKS(10)) );
  ESP_ERROR_CHECK( uart_set_pin(uartPort, TX_IO, RX_IO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) );
  ESP_ERROR_CHECK( gpio_pullup_dis((gpio_num_t) RX_IO) );
}

void SPort::sendResponse(const char msg[]) {
  char cks = cksum(msg, 0, 7);
  ESP_ERROR_CHECK( uart_set_pin(uartPort, RX_IO, TX_IO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) );
  ESP_ERROR_CHECK( gpio_pullup_dis((gpio_num_t) RX_IO) );
  for (int i=0; i<7; i++) {
    sendEncodedChar(msg[i]);
  }
  sendEncodedChar(cks);
  ESP_ERROR_CHECK( uart_wait_tx_done(uartPort, pdMS_TO_TICKS(10)) );
  ESP_ERROR_CHECK( uart_set_pin(uartPort, TX_IO, RX_IO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) );
  ESP_ERROR_CHECK( gpio_pullup_dis((gpio_num_t) RX_IO) );
}

uint32_t SPortMetric::convertLatLon(float latLon, bool isLat) {
  uint32_t data = (uint32_t)((latLon < 0 ? -latLon : latLon) * 60 * 10000) & 0x3FFFFFFF;
  if(isLat == false) {
    data |= 0x80000000;
  }
  if(latLon < 0) {
    data |= 0x40000000;
  }
  return data;
}

uint32_t SPortMetric::convertDateTime(uint8_t yearOrHour, uint8_t monthOrMinute, uint8_t dayOrSecond, bool isDate) {
  uint32_t data = yearOrHour;
  data <<= 8;
  data |= monthOrMinute;
  data <<= 8;
  data |= dayOrSecond;
  data <<= 8;
  if(isDate == true) {
    data |= 0xFF;
  }
  return data;
}

void SPortMetric::getFrame(char *frame, TinyGPSPlus* gps, Beacon *beacon) {
  frame[0] = 0x10;
  frame[1] = ID & 0xff;
  frame[2] = ID >> 8;
  uint32_t value = 0;
  switch (type) {
    case SPORT_GPS_METRIC_LAT:
      value = convertLatLon(gps->location.lat(), true);
      break;
    case SPORT_GPS_METRIC_LON:
      value = convertLatLon(gps->location.lng(), false);
      break;
    case SPORT_GPS_METRIC_ALT:
      value = (uint32_t) gps->altitude.meters() * 100;
      break;
    case SPORT_GPS_METRIC_SPEED:
      value = (uint32_t) gps->speed.knots() * 1000;
      break;
    case SPORT_GPS_METRIC_COG:
      value = gps->course.deg() * 100;
      break;
    case SPORT_GPS_METRIC_DATE:
      value = convertDateTime(gps->date.year() - 2000, gps->date.month(), gps->date.day(), true);
      break;
    case SPORT_GPS_METRIC_TIME:
      value = convertDateTime(gps->time.hour(), gps->time.minute(), gps->time.second(), false);
      break;
    case SPORT_BEACON_METRIC_STAT:
      value = beacon->getState();
      break;
    case SPORT_BEACON_METRIC_SAT:
      value = (uint32_t)(gps->satellites.value());
      break;
    case SPORT_BEACON_METRIC_HDOP:
      if (gps->hdop.hdop() < 0) {
        value = 9999;
      } else {
        value = (uint32_t)(gps->hdop.hdop() * 100);
      }
      break;
    case SPORT_BEACON_METRIC_PREFIX:
      value = beacon->getLastPrefix();
      break;
    case SPORT_BEACON_METRIC_FRAMES:
      value = beacon->getSentFramesCount();
      break;
    default:
      break;
  }
  frame[3] = value & 0xff;
  frame[4] = value >> 8 & 0xff;
  frame[5] = value >> 16 & 0xff;
  frame[6] = value >> 24 & 0xff;
  lastSentTS = millis();
}

bool SPortMetric::isTooSoon() {
  return ((millis() - lastSentTS) < periodMs);
}

void SPort::parseCommand(const char *msg, int len) {
  ESP_LOGV(TAG, "Potential command:");
  ESP_LOG_BUFFER_HEXDUMP(TAG, msg, len, ESP_LOG_VERBOSE);
  char cks = cksum(msg, 0, len - 1);
  if (cks != msg[len-1]) {
    ESP_LOGV(TAG, "CheckSum invalid");
    return;
  }
  ESP_LOGD(TAG, "CheckSum valid");
  if (msg[1] == 0xff && msg[2] == 0x00) {
    uint32_t bin_pref = msg[3] + (msg[4] << 8) + (msg[5] << 16) + (msg[6] << 24);
    uint8_t new_prefix[4];
    new_prefix[0] = '0' + bin_pref / 1000;
    uint32_t left_part = bin_pref % 1000;
    new_prefix[1] = '0' + left_part / 100;
    left_part %= 100;
    new_prefix[2] = '0' + left_part / 10;
    left_part %= 10;
    new_prefix[3] = '0' + left_part;
    ESP_LOGD(TAG, "New Prefix: %d=%c%c%c%c", bin_pref, new_prefix[0], new_prefix[1], new_prefix[2], new_prefix[3]);
    esp_err_t err = config->setPrefix(new_prefix);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Can't change prefix: %s", esp_err_to_name(err));
    }
    beacon->computeID();
  }
}

uint8_t SPort::findNextMetricToSendIndex() {
  uint8_t i = (lastBeaconMetricIndex + 1) % SPORT_GPS_METRIC_LAST;
  while (i != lastBeaconMetricIndex) {
    if (!gpsMetrics[i].isTooSoon()) {
      return i;
    }
    i = (i + 1) % SPORT_GPS_METRIC_LAST;
  }
  return SPORT_GPS_METRIC_LAST;
}

uint32_t SPort::readChars() {
  // reading pending / lost stuff
  size_t pending;
  uint32_t ts = 0;
  ESP_ERROR_CHECK( uart_get_buffered_data_len(uartPort, &pending ));
  int nb_chars = uart_read_bytes(uartPort, rxBuffer, RX_BUF_SIZE, pdMS_TO_TICKS(2));
  if (nb_chars >=2 ) {
    ts = millis();
    if(nb_chars > 2) {
      ESP_LOGD(TAG, "Read %d bytes (%d pending)", nb_chars, pending);
      ESP_LOG_BUFFER_HEXDUMP(TAG, rxBuffer, nb_chars, ESP_LOG_DEBUG);
    }
    for (int i = 0; i < nb_chars - 9; i++) {
      if (rxBuffer[i] == 0x7e &&
          rxBuffer[i + 1] == SPORT_SENSOR_ID_GPS_TX &&
          rxBuffer[i + 2] == 0x31) {
        parseCommand((const char *)&rxBuffer[i + 2], 8);
      }
    }
    if (rxBuffer[nb_chars - 2] != 0x7e || rxBuffer[nb_chars -1] != SPORT_SENSOR_ID_GPS) {
      return ts;
    }
    uint8_t next = findNextMetricToSendIndex();
    if (next == SPORT_GPS_METRIC_LAST) {
      ESP_LOGV(TAG, "GPS Sensor ID asked (0x%x 0x%x), but too soon for any metric", rxBuffer[nb_chars - 2], rxBuffer[nb_chars - 1]);
      sendTooSoon();
      return ts;
    }
    SPortMetric *metric = &gpsMetrics[next];
    ESP_LOGV(TAG, "GPS Sensor ID asked (0x%x 0x%x), returning %d/%d metric", rxBuffer[nb_chars - 2], rxBuffer[nb_chars - 1], next, SPORT_GPS_METRIC_LAST);
    char response[7];
    metric->getFrame(response, gps, beacon);
    sendResponse(response);
    lastBeaconMetricIndex = next;
  }
  return ts;
}

void SPort::handle(uint32_t end_ts) {
  while (millis() < end_ts) {
    uint32_t first_telem_char_ts = readChars();
    int32_t remaining_ms = 7 - (millis() - first_telem_char_ts);
    if (remaining_ms > 0) {
      ESP_LOGD(TAG, "I can sleep for %d ms, before next telem frame", remaining_ms);
      esp_sleep_enable_timer_wakeup(remaining_ms * uS_TO_mS_FACTOR);
      esp_light_sleep_start();
    }
  }
}