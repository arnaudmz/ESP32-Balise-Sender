// vim:et:sts=2:sw=2:si
#include <cstring>
#include "Beacon.h"

#include "esp_wifi.h"

//#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
//#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#define LOG_LOCAL_LEVEL ESP_LOG_INFO
static constexpr char TAG[] = "Beacon";
#include "esp_log.h"
#include "Config.h"
#include "Switches.h"
#include "TinyGPS++.h"
#include "GPSCnx.h"
#include "LED.h"
#include "droneID_FR.h"

#define WIFI_CHANNEL CONFIG_WIFI_CHANNEL

static constexpr uint8_t model_id_to_value[] = {1, 2, 3, 4};
static constexpr uint8_t mass_id_to_value[] = {2, 4, 25, 150};

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

}

void Beacon::begin() {
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

void Beacon::computeID() {
  if (switches->enabled()) {
    snprintf(droneIDStr, 33, "%3s%3s00000000%1d%03d%12s",
      config->getBuilder(),
      config->getVersion(),
      model_id_to_value[(switches->getGroupMSBState() << 1) + switches->getGroupLSBState()],
      mass_id_to_value[(switches->getMassMSBState() << 1) + switches->getMassLSBState()],
      config->getSuffix());
  } else {
    snprintf(droneIDStr, 33, "%3s%3s00000000%4s%12s",
      config->getBuilder(),
      config->getVersion(),
      config->getPrefix(),
      config->getSuffix());
  }
  ESP_LOGD(TAG, "Computed ID: %s", droneIDStr);
}

void Beacon::computeAndSendBeaconIfNeeded() {
  if (droneID->has_home_set() && droneID->time_to_send()) {
    led->toggleFade();
    float time_elapsed = (float(millis() - beaconSec) / 1000.0);
    beaconSec = millis();

    ESP_LOGI(TAG, "%.1fs Send beacon: (cause: %s) with %.1fm Speed=%.1f",
        time_elapsed, droneID->has_pass_distance() ? "Distance" : "Time",
        droneID->get_distance_from_last_position_sent(),
        droneID->get_ground_speed_kmh());
    if (switches->enabled()) {
      computeID();
      droneID->set_drone_id(droneIDStr);
    }
    const uint8_t to_send = droneID->generate_beacon_frame(beaconPacket, headerSize);
    sendBeacon(beaconPacket, to_send);
    droneID->set_last_send();
  }
}

void Beacon::handleData() {
  if (!gps->location.isValid()) {
    ESP_LOGI(TAG, "Positioning(%llu), valid: %d, hdop: %.2lf, time: %02d:%02d:%02d",
        gpsSec++,
        gps->satellites.value(),
        gps->hdop.hdop(),
        gps->time.hour(),
        gps->time.minute(),
        gps->time.second());
    led->blinkOnce();
  } else {
    if (!hasSetHome) {
      if (gps->satellites.value() > 6 && gps->hdop.hdop() < 2.0) {
        hasSetHome = true;
        homeAlt = gps->altitude.meters();
        ESP_LOGI(TAG, "Setting Home Position, Altitude=%d", (int)homeAlt);
        droneID->set_home_position(gps->location.lat(), gps->location.lng(), gps->altitude.meters());
      } else {
        // Looking for better precision to set home, blink twice
        led->blinkTwice();
      }
    }
    droneID->set_current_position(gps->location.lat(), gps->location.lng(), gps->altitude.meters());
    droneID->set_heading(gps->course.deg());
    droneID->set_ground_speed(gps->speed.mps());
    droneID->set_heigth(gps->altitude.meters() - homeAlt);
    ESP_LOGI(TAG, "%d:%02d:%02dZ: lng=%.4f, lat=%.4f, satt=%d, hdop=%.2f",
      gps->time.hour(),
      gps->time.minute(),
      gps->time.second(),
      gps->location.lng(),
      gps->location.lat(),
      gps->satellites.value(),
      gps->hdop.hdop());
    computeAndSendBeaconIfNeeded();
  }
}

