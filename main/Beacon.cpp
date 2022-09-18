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
#include <cstring>
#include "Beacon.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "Config.h"
#include "Switches.h"
#include "TinyGPS++.h"
#include "GPSCnx.h"
#include "LED.h"
#include "droneID_FR.h"
static constexpr char TAG[] = "Beacon";

#define WIFI_CHANNEL CONFIG_WIFI_CHANNEL

static constexpr uint8_t model_id_to_value[] = {1, 2, 3, 4};
static constexpr uint16_t mass_id_to_value[] = {2, 4, 25, 150, 999};

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

Beacon::Beacon(Config *c, LED *l, Switches *s, droneIDFR *d, TinyGPSPlus *g):
config(c),
led(l),
switches(s),
droneID(d),
gps(g) {
  const char *ssid = config->getSSID();
  const uint8_t ssid_size = strlen(ssid);
  beaconPacket[40] = strlen(ssid);  // set size
  memcpy(&beaconPacket[41], ssid, ssid_size); // set ssid
  memcpy(&beaconPacket[10], config->getMACAddr(), 6); // set mac as source
  memcpy(&beaconPacket[16], config->getMACAddr(), 6); // set mac as filter
  headerSize = 41 + ssid_size;
  if (! switches->enabled()) {
    computeID();
    droneID->set_drone_id(droneIDStr);
  }
  ESP_ERROR_CHECK( esp_netif_init() );
  ESP_ERROR_CHECK( esp_event_loop_create_default() );
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
  ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
  ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
}

void Beacon::sendBeacon(const uint8_t *packet, const uint8_t to_send) {
  ESP_LOGD(TAG, "Sending packet, %d bytes", to_send);
  ESP_LOG_BUFFER_HEXDUMP(TAG, packet, to_send, ESP_LOG_VERBOSE);
  ESP_ERROR_CHECK( esp_wifi_start() );
  ESP_ERROR_CHECK( esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE) );
  ESP_ERROR_CHECK( esp_wifi_80211_tx(WIFI_IF_STA, packet, to_send, true) );
  ESP_ERROR_CHECK( esp_wifi_stop() );
}

uint8_t Beacon::getGroupFromID(uint8_t g) {
  return model_id_to_value[g];
}

uint16_t Beacon::getMassFromID(uint8_t m) {
  return mass_id_to_value[m];
}

void Beacon::computeID() {
  if (switches->enabled()) {
    int gMSB = switches->getGroupMSBState();
    int gLSB = switches->getGroupLSBState();
    int mMSB = switches->getMassMSBState();
    int mLSB = switches->getMassLSBState();
    uint8_t model_group = getGroupFromID((gMSB << 1) + gLSB);
    uint16_t mass_group = getMassFromID((mMSB << 1) + mLSB);
    lastPrefix = model_group * 1000 + mass_group;
    lastPrefixStr[0] = '0' + model_group;
    lastPrefixStr[1] = '0' + mass_group / 100;
    lastPrefixStr[2] = '0' + (mass_group % 100) / 10;
    lastPrefixStr[3] = '0' + mass_group % 10;
    lastPrefixStr[4] = '\0';
  } else {
    strncpy(lastPrefixStr, config->getPrefix(), 5);
    lastPrefix = (lastPrefixStr[0] - '0') * 1000 +
               (lastPrefixStr[1] - '0') * 100 +
               (lastPrefixStr[2] - '0') * 10 +
               (lastPrefixStr[3] - '0');
  }
  snprintf(droneIDStr, 33, "%3s%3s00000000%4s%12s",
    config->getBuilder(),
    config->getVersion(),
    lastPrefixStr,
    config->getSuffix());
  droneID->set_drone_id(droneIDStr);
  ESP_LOGD(TAG, "Computed ID: %s", droneIDStr);
}

void Beacon::computeAndSendBeaconIfNeeded() {
  if (droneID->has_home_set() && droneID->time_to_send()) {
    led->toggleFade();
    float time_elapsed = (float(millis() - beaconSec) / 1000.0);
    beaconSec = millis();

    ESP_LOGI(TAG, "%.1fs Send beacon: (cause: %s) with %.1fm Speed=%.1f",
        time_elapsed, droneID->has_pass_distance() ? "Distance" : "Time",
        (float) droneID->get_distance_from_last_position_sent(),
        (float) droneID->get_ground_speed_kmh());
    if (switches->enabled()) {
      computeID();
    }
    const uint8_t to_send = droneID->generate_beacon_frame(beaconPacket, headerSize);
    sendBeacon(beaconPacket, to_send);
    droneID->set_last_send();
    sentFrames ++;
  }
}

BeaconState Beacon::getState() {
  if (!gps->location.isValid()) {
    return NO_GPS;
  }
  if (!hasSetHome) {
    return GPS_NOT_PRECISE;
  }
  if (!hasTakenOff) {
    return HAS_HOME;
  }
  return IN_FLIGHT;
}

void Beacon::handleData() {
  switch (getState()) {
    case NO_GPS:
      ESP_LOGI(TAG, "Positioning(%llu), valid: %d, hdop: %.2f, time: %02d:%02d:%02d",
          gpsSec++,
          gps->satellites.value(),
          (float) gps->hdop.hdop(),
          gps->time.hour(),
          gps->time.minute(),
          gps->time.second());
      led->blinkOnce();
      computeID(); // only needed for SPort Telemetry value to be correct ASAP
      return;
    case GPS_NOT_PRECISE:
      if (gps->satellites.value() >= config->getGPSSatThrs() && gps->hdop.hdop() <= (config->getGPSHDOPThrs() / 10.0)) {
        homeBestHDOP = gps->hdop.hdop();
        hasSetHome = true;
        homeAlt = gps->altitude.meters();
        ESP_LOGI(TAG, "Setting Home Position, Altitude=%d", (int)homeAlt);
        droneID->set_home_position(gps->location.lat(), gps->location.lng(), gps->altitude.meters());
      } else {
        // Looking for better precision to set home, blink twice
        led->blinkTwice();
      }
      computeID(); // only needed for SPort Telemetry value to be correct ASAP
      break;
    case HAS_HOME:
      if (gps->speed.kmph() > 10.0) {
        ESP_LOGI(TAG, "Take off");
        hasTakenOff = true;
      } else {
        double hdop = gps->hdop.hdop();
        if (hdop < homeBestHDOP) {
          homeAlt = gps->altitude.meters();
          ESP_LOGI(TAG, "Improving Home Position while not moving, Altitude=%d", (int)homeAlt);
          droneID->set_home_position(gps->location.lat(), gps->location.lng(), gps->altitude.meters());
          homeBestHDOP = hdop;
        }
      }
      break;
    default:
      break;
  }
  droneID->set_current_position(gps->location.lat(), gps->location.lng(), gps->altitude.meters());
  double course_deg = gps->course.deg();
  if(course_deg > 0.0 && course_deg < 360.0) {
    droneID->set_heading((uint16_t) course_deg);
  } else {
    droneID->set_heading(0);
  }
  droneID->set_ground_speed(gps->speed.mps());
  droneID->set_heigth(gps->altitude.meters() - homeAlt);
  ESP_LOGI(TAG, "%d:%02d:%02dZ: lng=%.4f, lat=%.4f, satt=%d, hdop=%.2f",
    gps->time.hour(),
    gps->time.minute(),
    gps->time.second(),
    (float) gps->location.lng(),
    (float) gps->location.lat(),
    gps->satellites.value(),
    (float) gps->hdop.hdop());
  computeAndSendBeaconIfNeeded();
}
