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
#include "JetiScreen.h"
#include <cstring>
#include <ctype.h>

//#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
//#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#define LOG_LOCAL_LEVEL ESP_LOG_INFO
static constexpr char TAG[] = "JetiScreen";
#include "esp_log.h"

JetiScreen::JetiScreen(Config *config, TinyGPSPlus *gps, Beacon *beacon): config(config), gps(gps), beacon(beacon) {
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

void JetiScreen::setHomeScreen() {
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
    
void JetiScreen::setBeaconIDScreen() {
  const char *id;
  char v_id[4], m_id[4];
  id = beacon->getLastID();
  strncpy(v_id, id, 3);
  v_id[3] = '\0';
  strncpy(m_id, &(id[3]), 3);
  m_id[3] = '\0';
  snprintf(screen, 33, "%3s %3s %24s", v_id, m_id, &(id[6]));
}

void JetiScreen::setSatStatusScreen() {
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

void JetiScreen::setRollerMenuScreen(bool isGroup) {
  if (isGroup) {
    snprintf(screen, 33, "*Nouveau Groupe*%16s", groups[newGroup]);
  } else {
    snprintf(screen, 33, "*Nouvelle Masse*%16s", masses[newMass]);
  }
}

uint8_t JetiScreen::getNextIndex(int8_t index, bool direction, uint8_t max) {
  index += direction ? 1 : -1;
  if (index < 0) {
    return max - 1;
  }
  if (index == max) {
    return 0;
  }
  return index;
}

void JetiScreen::prepareScreen() {
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

JetiMenuAction JetiScreen::navigate(JetiReply r) {
  if (r < JETI_REPLY_NONE) {
    return JETI_MENU_ACTION_NONE;
  }
  if ((millis() - lastButtonChangeTS) < 50) {
    ESP_LOGD(TAG, "Debounce (too soon)");
    return JETI_MENU_ACTION_NONE;
  }
  if (r == lastReply) {
    ESP_LOGD(TAG, "Debounce (same button(s))");
    return JETI_MENU_ACTION_NONE;
  }
  lastReply = r;
  lastButtonChangeTS = millis();
  switch(currentMenuItem) {
    case JETI_MENU_HOME:
      switch(r) {
        case JETI_REPLY_BUTTON_DOWN:
          currentMenuItem = JETI_MENU_BEACON_ID;
          break;
        case JETI_REPLY_BUTTON_UP:
          return JETI_MENU_ACTION_EXIT;
        default:
          break;
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
          return JETI_MENU_ACTION_NOTIFY;
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
          return JETI_MENU_ACTION_NOTIFY;
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
  return JETI_MENU_ACTION_NONE;
}

void JetiScreen::saveNewPrefix() {
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
}
