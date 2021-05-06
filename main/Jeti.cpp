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
#include "esp32/clk.h"

//#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
//#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#define LOG_LOCAL_LEVEL ESP_LOG_INFO
static constexpr char TAG[] = "Jeti";
#include "esp_log.h"

#define SPORT_BAUD_RATE 9700
#define RX_IO (gpio_num_t) 5
#define uS_TO_mS_FACTOR 1000
#define cycleWait(wait) for (uint32_t start = getCycleCount(); getCycleCount() - start < wait;)

JetiMetric metrics[JETI_METRIC_KIND_LAST] = {
  JetiMetric(JETI_METRIC_KIND_SENSOR_DESC, JETI_METRIC_TYPE_SENSOR_DESC, "LaBalise",   NULL),
  JetiMetric(JETI_METRIC_KIND_PREFIX,      JETI_METRIC_TYPE_INT14,       "Prefixe",    NULL),
  JetiMetric(JETI_METRIC_KIND_SPEED,       JETI_METRIC_TYPE_INT14,       "Vitesse",    "m/s"),
  JetiMetric(JETI_METRIC_KIND_ALT,         JETI_METRIC_TYPE_INT14,       "Altitude",   "m"),
  JetiMetric(JETI_METRIC_KIND_HDOP,        JETI_METRIC_TYPE_INT14,       "HDOP",       NULL),
  JetiMetric(JETI_METRIC_KIND_SAT,         JETI_METRIC_TYPE_INT6,        "Satellites", NULL)
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

#if 0
uint8_t JetiMetric::getDataLength() {
  // return data size + 1 (for id/datatype byte)
  switch (type) {
    case JETI_METRIC_TYPE_SENSOR_DESC:
      return 0;
    case JETI_METRIC_TYPE_INT6:
      return 2;
    case JETI_METRIC_TYPE_INT14:
    case JETI_METRIC_TYPE_INT14_RESERVED_2:
    case JETI_METRIC_TYPE_INT14_RESERVED_3:
      return 3;
    case JETI_METRIC_TYPE_INT22:
    case JETI_METRIC_TYPE_INT22_DATE_TIME:
    case JETI_METRIC_TYPE_INT22_RESERVED_6:
    case JETI_METRIC_TYPE_INT22_RESERVED_7:
      return 4;
    case JETI_METRIC_TYPE_INT30:
    case JETI_METRIC_TYPE_INT30_GPS:
    case JETI_METRIC_TYPE_INT30_RESERVED_10:
    case JETI_METRIC_TYPE_INT30_RESERVED_11:
      return 5;
    case JETI_METRIC_TYPE_INT38_RESERVED_12:
    case JETI_METRIC_TYPE_INT38_RESERVED_13:
    case JETI_METRIC_TYPE_INT38_RESERVED_14:
    case JETI_METRIC_TYPE_INT38_RESERVED_15:
      return 6;
  }
  return 0;
}
#endif

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
    default:
      break;
  }
  return pos;
}

JetiTelemetry::JetiTelemetry(Config *c, TinyGPSPlus *gps, Beacon *beacon):
config(c),
gps(gps),
beacon(beacon) {
  gpio_pad_select_gpio(RX_IO);
  bitTime = esp_clk_cpu_freq() / 9700;
  switchToRX();
  const char * pref = config->getPrefix();
  for (int i = 0; i < 4; i ++) {
    if (pref[0] - '0' == beacon->getGroupFromID(i)) {
      newGroup = i;
      break;
    }
  }
  uint16_t mass = (pref[1] -'0') * 100 + (pref[2] - '0') * 10 + (pref[3] - '0');
  for (int i = 0; i < 5; i ++) {
    if (mass == beacon->getMassFromID(i)) {
      newMass = i;
      break;
    }
  }
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
  uint32_t ccount;
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

void JetiTelemetry::sendText(const char *st) {
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

//#define BLINK gpio_set_level(GPIO_NUM_17, 1); gpio_set_level(GPIO_NUM_17, 0)

JetiReply JetiTelemetry::lookForReply() {
  uint32_t now = millis();
  while (millis() - now < 5) {
    if (gpio_get_level(RX_IO) == 0) { // start bit
      portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
      uint8_t computed_parity = 0, read_parity;
      uint8_t value = 0;
      bool bit8;
      portENTER_CRITICAL(&mux);
      //BLINK;
      cycleWait(bitTime + (bitTime >> 1)); // start bit
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

void JetiTelemetry::sendExAlarm(uint8_t b) {
  uartWrite(0x7E, false);
  uartWrite(0x52, true);
  uartWrite(0x23, true);
  uartWrite(b, true);
}

#if 0
uint8_t JetiTelemetry::writeAndUpdateCRC(uint8_t c, uint8_t crc_seed) {
  uartWrite(c, true);
  return updateCRC(c, crc_seed);
}

void JetiTelemetry::sendExMessage(const char *st) {
  uint8_t crc = 0;
  uint8_t len = strlen(st);
  uartWrite(0x7E, false);
  uartWrite(0x9F, true);
  crc = writeAndUpdateCRC((0x02 << 6) | (len + 8), crc);
  crc = writeAndUpdateCRC(0x11, crc); // manufactor ID MSB
  crc = writeAndUpdateCRC(0xA4, crc); // manufactor ID LSB
  crc = writeAndUpdateCRC(0x11, crc); // Device ID MSB
  crc = writeAndUpdateCRC(0xA4, crc); // Device ID LSB
  crc = writeAndUpdateCRC(0x00, crc); // Always 0
  crc = writeAndUpdateCRC(0x7f, crc); // Message type
  crc = writeAndUpdateCRC((0x01 << 5) | len, crc); // Message class (0 = basic info) + length
  for(int i = 0; i < len; i++) {
    crc = writeAndUpdateCRC(st[i], crc);
  }
  uartWrite(crc, true);
}
#endif

void JetiTelemetry::setHomeScreen() {
  char status[17];
  switch (beacon->getState())
  {
    case NO_GPS:
      strcpy(status, "No Fix");
      break;
    case GPS_NOT_PRECISE:
      strcpy(status, "No GPS Precision");
      break;
    case HAS_HOME:
      strcpy(status, "Prete, au sol");
      break;
    case IN_FLIGHT:
      strcpy(status, "En vol");
      break;
  }
  const char *pref = beacon->getLastPrefixStr();
  snprintf(screen, 33, "LaBalise  [%c%c%c%c]%16s", pref[0], pref[1], pref[2], pref[3], status);
}
    
void JetiTelemetry::setBeaconIDScreen() {
  const char *id;
  char v_id[4], m_id[4];
  id = beacon->getLastID();
  strncpy(v_id, id, 3);
  v_id[3] = '\0';
  strncpy(m_id, &(id[3]), 3);
  m_id[3] = '\0';
  snprintf(screen, 33, "%3s %3s %24s", v_id, m_id, &(id[6]));
}

void JetiTelemetry::setSatStatusScreen() {
  double hdop = gps->hdop.hdop();
  if(hdop <= 0.0) {
    hdop = 99.99;
  }
  if (hdop < 10.0) {
    snprintf(screen, 33, "Satellites :  %2dHDOP :      %1.2f", gps->satellites.value(), hdop);
  } else {
    snprintf(screen, 33, "Satellites :  %2dHDOP :     %2.2f", gps->satellites.value(), hdop);
  }
}

const char *groups[] = {
  "\x7f 1 - Captif   \x7e",
  "\x7f 2 - Planeur  \x7e",
  "\x7f 3 - Rotor    \x7e",
  "\x7f 4 - Avion    \x7e"
};

const char *masses[] = {
  "\x7f  0.8kg<m<2kg \x7e",
  "\x7f   2kg<m<4kg  \x7e",
  "\x7f  4kg<m<25kg  \x7e",
  "\x7f 25kg<m<150kg \x7e",
  "\x7f    m>150kg   \x7e"
};

void JetiTelemetry::setRollerMenuScreen(bool isGroup) {
  if (isGroup) {
    snprintf(screen, 33, "*Nouveau Groupe*%16s", groups[newGroup]);
  } else {
    snprintf(screen, 33, "*Nouvelle Masse*%16s", masses[newMass]);
  }
}

uint8_t JetiTelemetry::getNextIndex(int8_t index, bool direction, uint8_t max) {
  index += direction ? 1 : -1;
  if (index < 0) {
    return max - 1;
  }
  if (index == max) {
    return 0;
  }
  return index;
}

void JetiTelemetry::prepareScreen() {
  switch(currentMenuItem) {
    case JETI_MENU_HOME:
      setHomeScreen();
      break;
    case JETI_MENU_BEACON_ID:
      setBeaconIDScreen();
      break;
    case JETI_MENU_SAT_STATUS:
      setSatStatusScreen();
      break;
    case JETI_MENU_SETTINGS_GROUP_MENU:
      snprintf(screen, 33, "   *Reglages*    ID GROUPE [%c] \x7e", beacon->getLastPrefixStr()[0]);
      break;
    case JETI_MENU_SETTINGS_MASS_MENU:
      snprintf(screen, 33, "   *Reglages*   \x7fID MASSE [%c%c%c] ",
        beacon->getLastPrefixStr()[1],
        beacon->getLastPrefixStr()[2],
        beacon->getLastPrefixStr()[3]);
      break;
    case JETI_MENU_SETTINGS_GROUP_NEW:
      setRollerMenuScreen(true);
      break;
    case JETI_MENU_SETTINGS_MASS_NEW:
      setRollerMenuScreen(false);
      break;
    default:
      break;
  }
}

void JetiTelemetry::navigate(JetiReply r) {
  switch(currentMenuItem) {
    case JETI_MENU_HOME:
      if (r == JETI_REPLY_BUTTON_DOWN) {
        currentMenuItem = JETI_MENU_BEACON_ID;
      }
      break;
    case JETI_MENU_BEACON_ID:
      switch(r) {
        case JETI_REPLY_BUTTON_UP:
          currentMenuItem = JETI_MENU_HOME;
          break;
        case JETI_REPLY_BUTTON_DOWN:
          currentMenuItem = JETI_MENU_SAT_STATUS;
          break;
        default:
          break;
      }
      break;
    case JETI_MENU_SAT_STATUS:
      switch(r) {
        case JETI_REPLY_BUTTON_UP:
          currentMenuItem = JETI_MENU_BEACON_ID;
          break;
        case JETI_REPLY_BUTTON_DOWN:
          currentMenuItem = JETI_MENU_SETTINGS_GROUP_MENU;
          break;
        default:
          break;
      }
      break;
    case JETI_MENU_SETTINGS_GROUP_MENU:
      switch(r) {
        case JETI_REPLY_BUTTON_UP:
          currentMenuItem = JETI_MENU_SAT_STATUS;
          break;
        case JETI_REPLY_BUTTON_DOWN:
          currentMenuItem = JETI_MENU_SETTINGS_GROUP_NEW;
          break;
        case JETI_REPLY_BUTTON_RIGHT:
          currentMenuItem = JETI_MENU_SETTINGS_MASS_MENU;
          break;
        default:
          break;
      }
      break;
    case JETI_MENU_SETTINGS_MASS_MENU:
      switch(r) {
        case JETI_REPLY_BUTTON_UP:
          currentMenuItem = JETI_MENU_SAT_STATUS;
          break;
        case JETI_REPLY_BUTTON_DOWN:
          currentMenuItem = JETI_MENU_SETTINGS_MASS_NEW;
          break;
        case JETI_REPLY_BUTTON_LEFT:
          currentMenuItem = JETI_MENU_SETTINGS_GROUP_MENU;
          break;
        default:
          break;
      }
      break;
    case JETI_MENU_SETTINGS_GROUP_NEW:
      switch(r) {
        case JETI_REPLY_BUTTON_UP:
          currentMenuItem = JETI_MENU_SETTINGS_GROUP_MENU;
          break;
        case JETI_REPLY_BUTTON_DOWN:
          ESP_LOGD(TAG, "Save NEW GROUP!!!");
          saveNewPrefix();
          currentMenuItem = JETI_MENU_SETTINGS_GROUP_MENU;
          break;
        case JETI_REPLY_BUTTON_LEFT:
          newGroup = getNextIndex(newGroup, false, 4);
          break;
        case JETI_REPLY_BUTTON_RIGHT:
          newGroup = getNextIndex(newGroup, true, 4);
          break;
        default:
          break;
      }
      break;
    case JETI_MENU_SETTINGS_MASS_NEW:
      switch(r) {
        case JETI_REPLY_BUTTON_UP:
          currentMenuItem = JETI_MENU_SETTINGS_MASS_MENU;
          break;
        case JETI_REPLY_BUTTON_DOWN:
          ESP_LOGD(TAG, "Save NEW MASS!!!");
          saveNewPrefix();
          currentMenuItem = JETI_MENU_SETTINGS_MASS_MENU;
          break;
        case JETI_REPLY_BUTTON_LEFT:
          newMass = getNextIndex(newMass, false, 5);
          break;
        case JETI_REPLY_BUTTON_RIGHT:
          newMass = getNextIndex(newMass, true, 5);
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }
}

void JetiTelemetry::saveNewPrefix() {
  uint32_t bin_pref = beacon->getGroupFromID(newGroup) * 1000 + beacon->getMassFromID(newMass);
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
  notifyPrefChange = true;
}


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
  prepareScreen();
  while((millis() + 100) < end_ts) {
    switchToTX();
    if (notifyPrefChange) {
      sendExAlarm('C');
      notifyPrefChange = false;
    } else if (beacon->getState() > lastNotifiedState) {
      switch (beacon->getState()) {
        case GPS_NOT_PRECISE:
          sendExAlarm('I');
          break;
        case HAS_HOME:
          sendExAlarm('0');
          break;
        case IN_FLIGHT:
          sendExAlarm('C');
          break;
        default:
          break;
      }
      lastNotifiedState = beacon->getState();
    } else {
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
    }
    sendText(screen);
    switchToRX();
    esp_sleep_enable_timer_wakeup(3 * uS_TO_mS_FACTOR);
    esp_light_sleep_start();
    JetiReply r = lookForReply();
    if (r >= JETI_REPLY_NONE) {
      navigate(r);
//      if (r < lastReply) {
//        ESP_LOGD(TAG, "Button(s) released: %d", lastReply);
//        navigate(lastReply);
//      }
//      lastReply = r;
    }
    esp_sleep_enable_timer_wakeup(16 * uS_TO_mS_FACTOR);
    esp_light_sleep_start();
  }
}