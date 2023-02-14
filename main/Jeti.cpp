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
#include "Jeti.h"
#include <cstring>
#include <ctype.h>
#include <cmath>
#include "driver/gpio.h"
#include "esp_sleep.h"
#include "esp_private/esp_clk.h"
#include "driver/uart.h"
#include "esp_log.h"

#define RX_BUF_SIZE 1024
#define JETI_EXBUS_BAUD_RATE 125000

static constexpr char TAG[] = "Jeti";

#define RX_IO (gpio_num_t) 5
#define uS_TO_mS_FACTOR 1000
#define TX_IO (gpio_num_t) 18
#define cycleWait(wait) for (uint32_t start = getCycleCount(); getCycleCount() - start < wait;)

JetiMetric metrics[JETI_METRIC_KIND_LAST] = {
  JetiMetric(JETI_METRIC_KIND_SENSOR_DESC, JETI_METRIC_TYPE_SENSOR_DESC, "LaBalise",   NULL),
  JetiMetric(JETI_METRIC_KIND_PREFIX,      JETI_METRIC_TYPE_INT14,       "Prefixe",    NULL),
  JetiMetric(JETI_METRIC_KIND_SPEED,       JETI_METRIC_TYPE_INT14,       "Vitesse",    "m/s"),
  JetiMetric(JETI_METRIC_KIND_ALT,         JETI_METRIC_TYPE_INT14,       "Altitude",   "m"),
  JetiMetric(JETI_METRIC_KIND_HDOP,        JETI_METRIC_TYPE_INT14,       "HDOP",       NULL),
  JetiMetric(JETI_METRIC_KIND_SAT,         JETI_METRIC_TYPE_INT6,        "Satellites", NULL),
  JetiMetric(JETI_METRIC_KIND_STAT,        JETI_METRIC_TYPE_INT6,        "Statut",     NULL)
};

uint8_t JetiMetric::writeDesc(uint8_t *buffer, uint8_t pos) {
  uint8_t desc_len = description ? strlen(description) : 0;
  uint8_t unit_len = unit ? strlen(unit) : 0;
  buffer[pos++] = kind;
  buffer[pos++] = (desc_len << 3) | unit_len;
  for(int i = 0; i < desc_len; i++) {
    buffer[pos++] = description[i];
  }
  for(int i = 0; i < unit_len; i++) {
    buffer[pos++] = unit[i];
  }
  return pos;
}

uint8_t JetiMetric::formatINT6b(uint8_t *buffer, uint8_t pos, int8_t value, uint8_t decimals) {
    buffer[pos++] = (kind << 4) | type;
    buffer[pos++] = (value & 0x1F) | (value < 0 ? 0x80: 0x00) | (decimals << 5);
    return pos;
}

uint8_t JetiMetric::formatINT14b(uint8_t *buffer, uint8_t pos, int16_t value, uint8_t decimals) {
    buffer[pos++] = (kind << 4) | type;
    buffer[pos++] = value & 0xff;
    buffer[pos++] = ((value >> 8) & 0x1F) | (value < 0 ? 0x80: 0x00) | (decimals << 5);
    return pos;
}

uint8_t JetiMetric::writeData(uint8_t *buffer, uint8_t pos, TinyGPSPlus *gps, Beacon *beacon) {
  double hdop;
  switch(kind) {
    case JETI_METRIC_KIND_HDOP:
      hdop = gps->hdop.hdop();
      if (hdop <= 0.0) {
        hdop = 99.9;
      }
      pos = formatINT14b(buffer, pos, hdop * 10, 1);
      break;
    case JETI_METRIC_KIND_SPEED:
      pos = formatINT14b(buffer, pos, gps->speed.mps() * 10, 1);
      break;
    case JETI_METRIC_KIND_SAT:
      pos = formatINT6b(buffer, pos, gps->satellites.value(), 0);
      break;
    case JETI_METRIC_KIND_ALT:
      pos = formatINT14b(buffer, pos, gps->altitude.meters(), 0);
      break;
    case JETI_METRIC_KIND_PREFIX:
      pos = formatINT14b(buffer, pos, beacon->getLastPrefix(), 0);
      break;
    case JETI_METRIC_KIND_STAT:
      pos = formatINT6b(buffer, pos, (int8_t) beacon->getState(), 0);
      break;
    default:
      break;
  }
  return pos;
}

JetiTelemetry::JetiTelemetry(Config *c, TinyGPSPlus *gps, Beacon *beacon):
config(c),
gps(gps),
beacon(beacon),
screen(c, gps, beacon) {
  gpio_reset_pin(RX_IO);
  bitTime = esp_clk_cpu_freq() / 9700;
  switchToRX();
  //gpio_pad_select_gpio(GPIO_NUM_17);
  //ESP_ERROR_CHECK( gpio_set_direction(GPIO_NUM_17, GPIO_MODE_OUTPUT) );
}

void JetiTelemetry::switchToRX() {
  ESP_ERROR_CHECK( gpio_set_direction(RX_IO, GPIO_MODE_INPUT) );
  ESP_ERROR_CHECK( gpio_pullup_en(RX_IO) );
}

void JetiTelemetry::switchToTX() {
  ESP_ERROR_CHECK( gpio_pullup_dis(RX_IO) );
  ESP_ERROR_CHECK( gpio_set_direction(RX_IO, GPIO_MODE_OUTPUT) );
}

uint32_t JetiTelemetry::getCycleCount() {
  uint32_t ccount = 0;
  __asm__ __volatile__("esync; rsr %0,ccount":"=a" (ccount));
  return ccount;
}

void JetiTelemetry::uartWrite(uint8_t byte, bool bit8) {
  uint8_t parity = 0;
  portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
  portENTER_CRITICAL(&mux);

  // Start bit
  gpio_set_level(RX_IO, 0);
  cycleWait(bitTime);

  // 8 regular bits
  for (uint8_t i = 0; i != 8; i++) {
    if (byte & 1) {
      parity++;
      gpio_set_level(RX_IO, 1);
    } else {
      gpio_set_level(RX_IO, 0);
    }
    cycleWait(bitTime);
    byte >>= 1;
  }
  // 9th bit (command or data)
  if (bit8) {
    parity++;
    gpio_set_level(RX_IO, 1);
  } else {
    gpio_set_level(RX_IO, 0);
  }
  cycleWait(bitTime);
  // parity bit
  gpio_set_level(RX_IO, !(parity & 1));
  cycleWait(bitTime);
  // stop bits
  gpio_set_level(RX_IO, 1);
  cycleWait(bitTime);
  cycleWait(bitTime);

  portEXIT_CRITICAL(&mux);
}

#define POLY 0x07
uint8_t JetiTelemetry::updateCRC(uint8_t c, uint8_t crc_seed) {
  uint8_t crc_u = c;
  crc_u ^= crc_seed;
  for (int i = 0; i < 8; i++) {
    crc_u = ( crc_u & 0x80 ) ? POLY ^ ( crc_u << 1 ) : ( crc_u << 1 );
  }
  return crc_u;
}

void JetiTelemetry::sendJetiBoxScreen(const char *st) {
  ESP_LOGV(TAG, "Send screen: %s", st);
  uartWrite(0xFE, false);
  uint8_t len = strlen(st);
  for (int i = 0; i < 32; i++) {
    if (i < (32-len)) {
      uartWrite(' ', true);
    } else {
     uartWrite(st[i-(32 - len)], true);
    }
  }
  uartWrite(0xFF, false);
}

#define BLINK gpio_set_level(GPIO_NUM_17, 1); gpio_set_level(GPIO_NUM_17, 0)

JetiReply JetiTelemetry::lookForReply() {
  uint32_t now = millis();
  while (millis() - now < 10) {
    //BLINK;
    if (gpio_get_level(RX_IO) == 0) { // start bit
      portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
      uint8_t computed_parity = 0, read_parity;
      uint8_t value = 0;
      bool bit8;
      portENTER_CRITICAL(&mux);
      //BLINK;
      cycleWait(bitTime + (bitTime >> 4)); // start bit
      for (int i = 0; i < 8; i++) {
        if (gpio_get_level(RX_IO)) {
          computed_parity ++;
          value |= 1 << i;
        }
        //BLINK;
        cycleWait(bitTime); // i-th bit
      }
      if (gpio_get_level(RX_IO)) {
        computed_parity ++;
        bit8 = true;
      } else {
        bit8 = false;
      }
      //BLINK;
      cycleWait(bitTime); // bit 8
      read_parity = gpio_get_level(RX_IO);
      //BLINK;
      portEXIT_CRITICAL(&mux);
      if(read_parity == (computed_parity & 0x1)) {
        ESP_LOGV(TAG, "Invalid parity");
        return JETI_REPLY_INVALID;
      }
      if(bit8) {
        ESP_LOGV(TAG, "Invalid bit8, should be 0");
        return JETI_REPLY_INVALID;
      }
      if(value & 0xf) {
        ESP_LOGV(TAG, "Invalid 4 LSB (should be 0)");
        return JETI_REPLY_INVALID;
      }
      ESP_LOGV(TAG, "Valid command : 0x%x, L=%d, D=%d, U=%d, R=%d", value, !(value&0x80), !(value&0x40), !(value&0x20), !(value&0x10));
      return (JetiReply) ((~value) & 0xf0);
    }
  }
  ESP_LOGV(TAG, "Reply timeout");
  return JETI_REPLY_ABSENT; // timeout
}

#if 0
void JetiTelemetry::sendExAlarm(uint8_t b) {
  uartWrite(0x7E, false);
  uartWrite(0x52, true);
  uartWrite(0x23, true);
  uartWrite(b, true);
}

void JetiTelemetry::sendExMessage(const char *st) {
  uint8_t pos = 8;
  uint8_t len = strlen(st);
  memcpy(&exFrameBuffer[pos], st, len);
  pos += len;
  exFrameBuffer[0] = (0x02 << 6) | pos;
  exFrameBuffer[6] = 0xca; // Primary identifier of the message type??
  exFrameBuffer[7] = (1 << 5) | len;
  uint8_t crc = 0;
  for(int i = 0; i < pos; i++) {
    crc = updateCRC(exFrameBuffer[i], crc);
  }
  exFrameBuffer[pos++] = crc;
  uartWrite(0x7E, false);
  uartWrite(0x9F, true);
  for(int i = 0; i < pos; i++) {
    uartWrite(exFrameBuffer[i], true);
  }
}
#endif

void JetiTelemetry::sendExMetricDesc() {
  uint8_t pos = 6;
  pos = metrics[sensor_id].writeDesc(exFrameBuffer, pos);
  exFrameBuffer[0] = (0x00 << 6) | pos;
  uint8_t crc = 0;
  for(int i = 0; i < pos; i++) {
    crc = updateCRC(exFrameBuffer[i], crc);
  }
  exFrameBuffer[pos++] = crc;
  uartWrite(0x7E, false);
  uartWrite(0x9F, true);
  for(int i = 0; i < pos; i++) {
    uartWrite(exFrameBuffer[i], true);
  }
}

void JetiTelemetry::sendExMetricData() {
  uint8_t pos = 6;
  // few items to send, they all fit in a single frame
  for (int i = JETI_METRIC_KIND_SENSOR_DESC + 1; i < JETI_METRIC_KIND_LAST; i++) {
    pos = metrics[i].writeData(exFrameBuffer, pos, gps, beacon);
  }
  exFrameBuffer[0] = (0x01 << 6) | pos;
  uint8_t crc = 0;
  for(int i = 0; i < pos; i++) {
    crc = updateCRC(exFrameBuffer[i], crc);
  }
  exFrameBuffer[pos++] = crc;
  uartWrite(0x7E, false);
  uartWrite(0x9F, true);
  for(int i = 0; i < pos; i++) {
    uartWrite(exFrameBuffer[i], true);
  }
}

void JetiTelemetry::handle(uint32_t end_ts) {
  screen.prepareScreen();
  while((millis() + 150) < end_ts) {
    switchToTX();
    if (send_desc) {
      sendExMetricDesc();
    } else {
      sendExMetricData();
    }
    sensor_id++;
    if (sensor_id == JETI_METRIC_KIND_LAST) {
      sensor_id = JETI_METRIC_KIND_SENSOR_DESC;
      send_desc = !send_desc;
    }
    sendJetiBoxScreen(screen.getScreen());
    switchToRX();
    JetiReply r = lookForReply();
    if (r >= JETI_REPLY_NONE) {
      screen.navigate(r); // ignore return value (TODO: no notification so far)
    }
    esp_sleep_enable_timer_wakeup(50 * uS_TO_mS_FACTOR);
    esp_light_sleep_start();
  }
}

JetiExBusTelemetry::JetiExBusTelemetry(Config *c, TinyGPSPlus *gps, Beacon *beacon):
JetiTelemetry(c, gps, beacon, JetiScreen(c, gps, beacon)) {
  uart_config_t uart_config = {
    .baud_rate = JETI_EXBUS_BAUD_RATE,
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
  ESP_ERROR_CHECK( gpio_pullup_dis(RX_IO) );
}

uint16_t JetiExBusTelemetry::updateCRC16_CCITT(uint16_t crc, uint8_t data) {
  uint16_t ret_val;
  data ^= (uint8_t)(crc) & (uint8_t)(0xFF);
  data ^= data << 4;
  ret_val = ((((uint16_t)data << 8) | ((crc & 0xFF00) >> 8)) ^ (uint8_t)(data >> 4) ^ ((uint16_t)data << 3));
  return ret_val;
}

bool JetiExBusTelemetry::validateCRC16_CCITT(const uint8_t *buffer, uint8_t len) {
  uint16_t crc = 0;
	for (int i = 0; i < len - 2; i++) {
		crc = updateCRC16_CCITT(crc, buffer[i]);
  }
  return (buffer[len-2] == (crc & 0xff)) && (buffer[len-1] == (crc >> 8));
}

void JetiExBusTelemetry::sendResponse(const uint8_t *buffer, uint8_t len) {
  ESP_ERROR_CHECK( uart_set_pin(uartPort, RX_IO, TX_IO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) );
  ESP_ERROR_CHECK( gpio_pullup_dis((gpio_num_t) RX_IO) );
  uart_write_bytes(uartPort, (const char *) buffer, len);
  ESP_ERROR_CHECK( uart_wait_tx_done(uartPort, pdMS_TO_TICKS(10)) );
  ESP_ERROR_CHECK( uart_set_pin(uartPort, TX_IO, RX_IO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) );
  ESP_ERROR_CHECK( gpio_pullup_dis((gpio_num_t) RX_IO) );
  ESP_LOG_BUFFER_HEXDUMP(TAG, buffer, len, ESP_LOG_VERBOSE);
}

bool JetiExBusTelemetry::sendNextTelemetryFrame(uint8_t id) {
  exBusFrameTelemetryBuffer[3] = id;
  uint8_t pos = 13;
  if (send_desc) {
    pos = metrics[sensor_id].writeDesc(exBusFrameTelemetryBuffer, pos);
    exBusFrameTelemetryBuffer[7] = (0x00 << 6) | (pos - 7);
  } else {
    for (int i = JETI_METRIC_KIND_SENSOR_DESC + 1; i < JETI_METRIC_KIND_LAST; i++) {
      pos = metrics[i].writeData(exBusFrameTelemetryBuffer, pos, gps, beacon);
    }
    exBusFrameTelemetryBuffer[7] = (0x01 << 6) | (pos - 7);
  }
  exBusFrameTelemetryBuffer[2] = pos + 3;
  exBusFrameTelemetryBuffer[5] = pos - 5;
  uint8_t crc = 0;
  for(int i = 7; i < pos; i++) {
    crc = updateCRC(exBusFrameTelemetryBuffer[i], crc);
  }
  exBusFrameTelemetryBuffer[pos++] = crc;
  uint16_t crc16 = 0;
  for (int i = 0; i < pos; i++) {
		crc16 = updateCRC16_CCITT(crc16, exBusFrameTelemetryBuffer[i]);
  }
  exBusFrameTelemetryBuffer[pos++] = crc16 & 0xff;
  exBusFrameTelemetryBuffer[pos++] = crc16 >> 8;
  sendResponse(exBusFrameTelemetryBuffer, pos);
  sensor_id++;
  if (sensor_id == JETI_METRIC_KIND_LAST) {
    sensor_id = JETI_METRIC_KIND_SENSOR_DESC;
    send_desc = !send_desc;
  }
  return true;
}

void JetiExBusTelemetry::sendScreen(uint8_t id) {
  exBusFrameJetiBoxMenuBuffer[3] = id;
  memcpy(&exBusFrameJetiBoxMenuBuffer[6], screen.getScreen(), 32);
  uint8_t pos = 38;
  uint16_t crc16 = 0;
  for (int i = 0; i < pos; i++) {
		crc16 = updateCRC16_CCITT(crc16, exBusFrameJetiBoxMenuBuffer[i]);
  }
  exBusFrameJetiBoxMenuBuffer[pos++] = crc16 & 0xff;
  exBusFrameJetiBoxMenuBuffer[pos++] = crc16 >> 8;
  sendResponse(exBusFrameJetiBoxMenuBuffer, pos);
}

FrameRequestResult JetiExBusTelemetry::isLastFrameValid(const uint8_t *buffer, uint8_t nb_chars, uint8_t len, uint8_t request) {
    if (buffer[nb_chars - len] == 0x3d &&           // ExBusFrame…
        buffer[nb_chars - (len - 1)] == 0x01 &&     // That allows an answer
        buffer[nb_chars - (len - 2)] == len &&      // frame length as expected
        buffer[nb_chars - (len - 4)] == request) {  // frame requesting for expected request id
      if (!validateCRC16_CCITT(&buffer[nb_chars - len], len)) {
        ESP_LOGD(TAG, "Invalid Request Frame");
        return FRAME_INVALID;
      }
      return FRAME_PRESENT;
    }
    return FRAME_ABSENT;
}

uint32_t JetiExBusTelemetry::readChars() {
  // reading pending / lost stuff
  size_t pending;
  uint32_t ts = 0;
  ESP_ERROR_CHECK( uart_get_buffered_data_len(uartPort, &pending ));
  int nb_chars = uart_read_bytes(uartPort, rxBuffer, RX_BUF_SIZE, pdMS_TO_TICKS(2));
  if (nb_chars >=2 ) {
    ts = millis();
    if(nb_chars > 2) {
      ESP_LOGV(TAG, "Read %d bytes (%d pending)", nb_chars, pending);
      ESP_LOG_BUFFER_HEXDUMP(TAG, rxBuffer, nb_chars, ESP_LOG_VERBOSE);
    }
    if (isLastFrameValid(rxBuffer, nb_chars, 8, 0x3a) == FRAME_PRESENT) {
      uint8_t frame_id = rxBuffer[nb_chars - 5];
      ESP_LOGV(TAG, "Valid Request for Telemetry");
      cycleWait(esp_clk_cpu_freq() / 450); // to get around 4ms
      sendNextTelemetryFrame(frame_id);
      return ts;
    }
    if (isLastFrameValid(rxBuffer, nb_chars, 9, 0x3b) == FRAME_PRESENT) {
      uint8_t frame_id = rxBuffer[nb_chars - 6];
      uint8_t buttons = rxBuffer[nb_chars - 3];
      ESP_LOGV(TAG, "Valid Request for JetiBox Screen, buttons : 0x%x, L=%d, D=%d, U=%d, R=%d", buttons, !(buttons&0x80), !(buttons&0x40), !(buttons&0x20), !(buttons&0x10));
      switch (screen.navigate((JetiReply) ((~buttons) & 0xf0))) {
        case JETI_MENU_ACTION_NONE:
          cycleWait(esp_clk_cpu_freq() / 400); // to get around 4ms
          screen.prepareScreen();
          sendScreen(frame_id);
          break;
        case JETI_MENU_ACTION_NOTIFY:
          // TODO
          break;
        default:
          break;
      }
    }
  }
  return ts;
}

void JetiExBusTelemetry::handle(uint32_t end_ts) {
  while (millis() < end_ts) {
    uint32_t first_telem_char_ts = readChars();
    int32_t remaining_ms = 15 - (millis() - first_telem_char_ts);
    if (remaining_ms > 0) {
      ESP_LOGV(TAG, "I can sleep for %ld ms, before next telem frame", remaining_ms);
      esp_sleep_enable_timer_wakeup(remaining_ms * uS_TO_mS_FACTOR);
      esp_light_sleep_start();
    }
  }
}
