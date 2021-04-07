// vim:et:sts=2:sw=2:si
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
#define SPORT_GPS_ID 0x12


SPortMetric metrics[SPORT_METRIC_LAST] = {
//    SPortMetric(SPORT_METRIC_LAT,    1000,  SPORT_GPS_METRIC_LAT_LON_ID),
//    SPortMetric(SPORT_METRIC_LON,    1000,  SPORT_GPS_METRIC_LAT_LON_ID),
    SPortMetric(SPORT_METRIC_ALT,    1000,  SPORT_GPS_METRIC_ALT_ID),
    SPortMetric(SPORT_METRIC_SPEED,  1000,  SPORT_GPS_METRIC_SPEED_ID),
//    SPortMetric(SPORT_METRIC_COG,    1000,  SPORT_GPS_METRIC_COG_ID),
//    SPortMetric(SPORT_METRIC_DATE,   10000, SPORT_GPS_METRIC_TS_ID),
//    SPortMetric(SPORT_METRIC_TIME,   1000,  SPORT_GPS_METRIC_TS_ID),
    SPortMetric(SPORT_METRIC_STAT,   1000,  SPORT_BEACON_METRIC_STAT_ID),
    SPortMetric(SPORT_METRIC_SAT,    1000,  SPORT_BEACON_METRIC_SAT_ID),
    SPortMetric(SPORT_METRIC_HDOP,   1000,  SPORT_BEACON_METRIC_HDOP_ID),
    SPortMetric(SPORT_METRIC_PREFIX, 3000,  SPORT_BEACON_METRIC_PREFIX_ID),
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

void SPortMetric::getFrame(char *frame, TinyGPSPlus* gps, Beacon *beacon) {
  frame[0] = 0x10;
  frame[1] = ID & 0xff;
  frame[2] = ID >> 8;
  uint32_t value = 0;
  switch (type) {
//    case SPORT_METRIC_LAT:
//      break;
//    case SPORT_METRIC_LON:
//      break;
    case SPORT_METRIC_ALT:
      value = (uint32_t) gps->altitude.meters() * 100;
      break;
    case SPORT_METRIC_SPEED:
      value = (uint32_t) gps->speed.knots() * 1000;
      break;
//    case SPORT_METRIC_COG:
//      break;
//    case SPORT_METRIC_DATE:
//      break;
//    case SPORT_METRIC_TIME:
//      break;
    case SPORT_METRIC_STAT:
      value = beacon->getState();
      break;
    case SPORT_METRIC_SAT:
      value = (uint32_t)(gps->satellites.value());
      break;
    case SPORT_METRIC_HDOP:
      if (gps->hdop.hdop() < 0) {
        value = 9999;
      } else {
        value = (uint32_t)(gps->hdop.hdop() * 100);
      }
      break;
    case SPORT_METRIC_PREFIX:
      value = beacon->getLastPrefix();
      break;
    default:
      break;
  }
  frame[3] = value & 0xff;
  frame[4] = value >> 8 & 0xff;
  frame[5] = value >> 16 & 0xff;
  frame[6] = value >> 24 & 0xff;
}

void SPort::parseCommand(const char *msg, int len) {
  ESP_LOGV(TAG, "Potential command:");
  ESP_LOG_BUFFER_HEXDUMP(TAG, msg, len, ESP_LOG_INFO);
  char cks = cksum(msg, 0, len - 1);
  if (cks != msg[len-1]) {
    ESP_LOGV(TAG, "CheckSum invalid");
    return;
  }
  ESP_LOGV(TAG, "CheckSum valid");
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

void SPort::lookForCommand() {
  // reading pending / lost stuff
  int nb_chars = uart_read_bytes(uartPort, rxBuffer, RX_BUF_SIZE, pdMS_TO_TICKS(1));
  for (int i = 0; i < nb_chars - 10; i++) {
    if (rxBuffer[i] == 0x7e &&
        (rxBuffer[i+1] & 0x1f) == SPORT_GPS_ID &&
        rxBuffer[i+2] == 0x31) {
          parseCommand((const char*) &rxBuffer[i+2], 8);
    }
  }
}

bool SPort::waitForSPort() {
  lookForCommand();
  for (int i=0; i< 28; i++) {
    int nb_chars = uart_read_bytes(uartPort, rxBuffer, 1, pdMS_TO_TICKS(13));
    if (nb_chars == 1) {
      nb_chars += uart_read_bytes(uartPort, &rxBuffer[1], 16, pdMS_TO_TICKS(1));
      if (nb_chars >= 2 && rxBuffer[nb_chars-2] == 0x7e && (rxBuffer[nb_chars-1] & 0x1f) == SPORT_GPS_ID ) {
        ESP_LOGD(TAG, "Got: command for ME at attempt %d", i);
        return true;
      }
    } else {
      ESP_LOGI(TAG, "Failed to get SPort command, timeout");
      return false;
    }
  }
  ESP_LOGI(TAG, "Failed to get SPort command for ME, timeout");
  return false;
}

void SPort::readChars() {
  ESP_LOGV(TAG, "ReadChars...");
  char response[7];
  for (int i=0; i < SPORT_METRIC_LAST; i++) {
    if (waitForSPort()) {
      SPortMetric *metric = &metrics[i];
      metric->getFrame(response, gps, beacon);
      ESP_LOG_BUFFER_HEXDUMP(TAG, response, 7, ESP_LOG_VERBOSE);
      sendResponse(response);
    } else {
      return;
    }
  }
}
