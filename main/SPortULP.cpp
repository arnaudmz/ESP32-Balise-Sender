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
#include "SPortULP.h"
#include <ctype.h>
#include <cmath>
#include "esp_log.h"
#include "ulp_main.h"
#include "ulp_shared_types.h"

static constexpr char TAG[] = "SPortULP";

SPortULP::SPortULP(Config *c, TinyGPSPlus *gps, Beacon *beacon):
config(c),
gps(gps),
beacon(beacon) {
}

uint32_t SPortULP::convertLatLon(float latLon, bool isLat) {
  uint32_t data = (uint32_t)((latLon < 0 ? -latLon : latLon) * 60 * 10000) & 0x3FFFFFFF;
  if(isLat == false) {
    data |= 0x80000000;
  }
  if(latLon < 0) {
    data |= 0x40000000;
  }
  return data;
}

uint32_t SPortULP::convertDateTime(uint8_t yearOrHour, uint8_t monthOrMinute, uint8_t dayOrSecond, bool isDate) {
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

void SPortULP::handle(uint32_t end_ts) {
  ESP_LOGV(TAG, "SPortULP handler");
  ulp_millis = millis();
  if(ulp_metric_beacon_cmd_pending != 0) {
    if(ulp_metric_beacon_cmd == SPORT_CMD_NEW_PREFIX) {
      uint32_t bin_pref = ulp_metric_beacon_new_prefix;
      ESP_LOGV(TAG, "NEW PREFIX=%ld", bin_pref);
      uint8_t new_prefix[4];
      new_prefix[0] = '0' + bin_pref / 1000;
      uint32_t left_part = bin_pref % 1000;
      new_prefix[1] = '0' + left_part / 100;
      left_part %= 100;
      new_prefix[2] = '0' + left_part / 10;
      left_part %= 10;
      new_prefix[3] = '0' + left_part;
      ESP_LOGD(TAG, "New Prefix: %ld=%c%c%c%c", bin_pref, new_prefix[0], new_prefix[1], new_prefix[2], new_prefix[3]);
      esp_err_t err = config->setPrefix(new_prefix);
      if (err != ESP_OK) {
        ESP_LOGE(TAG, "Can't change prefix: %s", esp_err_to_name(err));
      }
      beacon->computeID();
    } else {
      ESP_LOGE(TAG, "Unknown pending cmd=%ld", ulp_metric_beacon_cmd);
    }
    ulp_metric_beacon_cmd_pending = 0;
  }
  ulp_metric_gps_alt = (uint32_t) gps->altitude.meters() * 100;
  ulp_metric_gps_cog = gps->course.deg() * 100;
  ulp_metric_gps_speed = (uint32_t) gps->speed.knots() * 1000;
  ulp_metric_gps_lat = convertLatLon(gps->location.lat(), true) ;
  ulp_metric_gps_lon = convertLatLon(gps->location.lng(), false);;
  ulp_metric_gps_date = convertDateTime(gps->date.year() - 2000, gps->date.month(), gps->date.day(), true);
  ulp_metric_gps_time = convertDateTime(gps->time.hour(), gps->time.minute(), gps->time.second(), false);
  ulp_metric_beacon_frames = beacon->getSentFramesCount();;
  ulp_metric_beacon_hdop = (gps->hdop.hdop() < 0)? 9999: (uint32_t)(gps->hdop.hdop() * 100);
  ulp_metric_beacon_prefix = beacon->getLastPrefix();
  ulp_metric_beacon_sat = (uint32_t)(gps->satellites.value());
  ulp_metric_beacon_stat = beacon->getState();
}
