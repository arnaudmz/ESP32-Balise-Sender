// vim:et:sts=2:sw=2:si
#ifndef __Beacon_h
#define __Beacon_h
#include "nvs.h"
#include "Config.h"
#include "LED.h"
#include "Switches.h"
#include "droneID_FR.h"
#include "TinyGPS++.h"

class Beacon {
  public:
    Beacon(Config *c, LED *l, Switches *s, droneIDFR *d, TinyGPSPlus *g);
    void handleData();
    bool hasSetHomeYet() { return hasSetHome; }

  private:
    void sendBeacon(const uint8_t *packet, const uint8_t to_send);
    void computeAndSendBeaconIfNeeded();
    void computeID();
    char droneIDStr[33];
    Config *config;
    LED *led;
    Switches *switches;
    droneIDFR *droneID;
    TinyGPSPlus *gps;
    uint8_t headerSize;
    bool hasSetHome = false;
    double homeAlt = 0.0;
    uint64_t beaconSec = 0;
    uint64_t gpsSec = 0;
};
#endif //ifndef __Beacon_h
