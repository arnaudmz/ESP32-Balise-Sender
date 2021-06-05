// vim:et:sts=2:sw=2:si
#include <esp_wifi.h>
#include <esp_event_loop.h>
#include <nvs_flash.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <Ticker.h>
#include <esp_adc_cal.h>
#include <esp_sleep.h>
#include <driver/rtc_io.h>
#include "Free_Fonts.h"
#include "Button2.h"


#define ADC_EN              14  //ADC_EN is the ADC detection enable port
#define ADC_PIN             34
#define BUTTON_1            35
#define BUTTON_2            0

String trame;
TFT_eSPI tft = TFT_eSPI(135, 240);
Ticker displayTicker;
Ticker battTicker;
Button2 btn1(BUTTON_1);
Button2 btn2(BUTTON_2);

int vref = 1100;
uint32_t last_battery_check_millis = 0;
uint32_t last_frame_ts = 0;
float battery_voltage = 0.0;
double last_displayed_batt_voltage = 0.0;

#define NB_PAGES 3
uint8_t page = 0;
uint8_t last_displayed_page = NB_PAGES;
bool must_display = true;
bool must_clean_display = true;


/**
  * Enumeration des types de données dans la trame
*/
enum DATA_TYPE: uint8_t {
  RESERVED = 0,
  PROTOCOL_VERSION = 1,
  ID_FR = 2,
  ID_ANSI_CTA = 3,
  LATITUDE = 4,        // In WS84 in degree * 1e5
  LONGITUDE = 5,       // In WS84 in degree * 1e5
  ALTITUDE = 6,        // In MSL in m
  HEIGHT = 7,          // From Home in m
  HOME_LATITUDE = 8,   // In WS84 in degree * 1e5
  HOME_LONGITUDE = 9,  // In WS84 in degree * 1e5
  GROUND_SPEED = 10,   // In m/s
  HEADING = 11,        // Heading in degree from north 0 to 359.
  NOT_DEFINED_END = 12,
};

/**
  * Tableau TLV (TYPE, LENGTH, VALUE) avec les tailles attendu des différents type données.
***/
static constexpr uint8_t TLV_LENGTH[] {
  0,  // [DATA_TYPE::RESERVED]
  1,  // [DATA_TYPE::PROTOCOL_VERSION]
  30, // [DATA_TYPE::ID_FR]
  0,  // [DATA_TYPE::ID_ANSI_CTA]
  4,  // [DATA_TYPE::LATITUDE]
  4,  // [DATA_TYPE::LONGITUDE]
  2,  // [DATA_TYPE::ALTITUDE]
  2,  // [DATA_TYPE::HEIGHT]
  4,  // [DATA_TYPE::HOME_LATITUDE]
  4,  // [DATA_TYPE::HOME_LONGITUDE]
  1,  // [DATA_TYPE::GROUND_SPEED]
  2,  // [DATA_TYPE::HEADING]
};


typedef struct  {
  int16_t fctl;
  int16_t duration;
  uint8_t da;
  uint8_t sa;
  uint8_t bssid;
  int16_t seqctl;
  unsigned char payload[];
} __attribute__((packed)) WifiMgmtHdr;

typedef struct {
  WifiMgmtHdr hdr;
  uint8_t payload[0];
} wifi_ieee80211_packet_t;

typedef struct {
  char ID[31];
  double lat;
  double lon;
  int alt;
  int height;
  int speed;
  int hdg;
  int rssi;
  double dep_lat;
  double dep_lon;
  uint32_t ts;
  char addr1[19]; /* receiver address */
  char addr2[19]; /* sender address */
  char addr3[19]; /* filtering address */
  char ssid[32];
} beacon_data_t;

beacon_data_t bdt;

static esp_err_t event_handler(void *ctx, system_event_t *event);
static void wifi_sniffer_init(void);
static void wifi_sniffer_set_channel(uint8_t channel);
static const char *wifi_sniffer_packet_type2str(wifi_promiscuous_pkt_type_t type);
static void wifi_sniffer_packet_handler(void *buff, wifi_promiscuous_pkt_type_t type);

esp_err_t event_handler(void *ctx, system_event_t *event) {
  return ESP_OK;
}

void wifi_sniffer_init(void) {
  nvs_flash_init();
  tcpip_adapter_init();
  ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
  ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
  ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_NULL) );
  ESP_ERROR_CHECK( esp_wifi_start() );
  ESP_ERROR_CHECK( esp_wifi_set_promiscuous(true) );
  ESP_ERROR_CHECK( esp_wifi_set_promiscuous_rx_cb(&beaconCallback) );
  ESP_ERROR_CHECK( esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE) );
}


// Function to extract MAC addr from a packet at given offset
void getMAC(char *addr, uint8_t* data, uint16_t offset) {
  sprintf(addr, "%02X:%02X:%02X:%02X:%02X:%02X", data[offset+0], data[offset+1], data[offset+2], data[offset+3], data[offset+4], data[offset+5]);
}

static int convertToInt(uint16_t start, int len, uint16_t size, uint8_t* data) {
  uint8_t count = size-1;
  int data_value = 0;
  bool neg_number = data[start] > 0x7F;

  for(uint16_t i = start; i < len && i < start+size; i++) {
    data_value +=  (data[i]) << (8 * count);
    count--;
  }

  if(neg_number) {
    data_value = (0xFFFF & ~data_value) + 1;
    data_value *= -1;
  }
  return data_value;
}

static void printDataSpan(uint16_t start, int len, uint16_t size, uint8_t* data) {
  for(uint16_t i = start; i < len && i < start+size; i++) {
    Serial.write(data[i]);
    trame = trame + char(data[i]);
  }
}

static double printCoordinates(uint16_t start, int len, uint16_t size, uint8_t* data) {
  uint8_t count = size-1;
  int data_value = 0;
  //Serial.print(" data_value="); Serial.print(data_value); Serial.print(" neg=");
  bool neg_number = data[start] > 0x7F;
  //Serial.print(neg_number);Serial.print(" ");

  for(uint16_t i = start; i < len && i < start+size; i++) {
    //Serial.print(count); Serial.print("-");
    data_value +=  (data[i]) << (8 * count);
    count--;
  }

  if(neg_number) {
    data_value = (0xFFFFFFFF & ~data_value) + 1;
    data_value *= -1;
  }
  double val = double(data_value) * 0.00001;
  Serial.print(val, 5);
  trame += String(val, 5);
  return val;
}

static void printAltitude(uint16_t start, int len, uint16_t size, uint8_t* data) {
  uint8_t count = size-1;
  int data_value = 0;
  bool neg_number = data[start] > 0x7F;
  //Serial.print(neg_number);Serial.print(" ");

  for(uint16_t i = start; i < len && i < start+size; i++) {
    //Serial.print(count); Serial.print("-");
    data_value +=  (data[i]) << (8 * count);
    count--;
  }

  if(neg_number) {
    data_value = (0xFFFF & ~data_value) + 1;
    data_value *= -1;
  }

  Serial.print(data_value);
  trame = trame + String(data_value);
}

void beaconCallback(void* buf, wifi_promiscuous_pkt_type_t type)
{
  wifi_promiscuous_pkt_t *snifferPacket = (wifi_promiscuous_pkt_t*)buf;
  WifiMgmtHdr *frameControl = (WifiMgmtHdr*)snifferPacket->payload;
  wifi_pkt_rx_ctrl_t ctrl = (wifi_pkt_rx_ctrl_t)snifferPacket->rx_ctrl;
  int len = snifferPacket->rx_ctrl.sig_len;
  uint8_t SSID_length = (int)snifferPacket->payload[40];
  uint8_t offset_OUI = 42 + SSID_length;
  //Organizationally Unique Identifier = 6A:5C:35 = Secrétariat général de la défense et de la sécurité nationale
  const uint8_t FRAME_OUI[3] = {0x6A, 0x5C, 0x35};

  //Filter OUI from 6A:5C:35
  if(snifferPacket->payload[offset_OUI+1] != FRAME_OUI[0] && snifferPacket->payload[offset_OUI+2] != FRAME_OUI[1] && snifferPacket->payload[offset_OUI+3] != FRAME_OUI[2])
  return;

  len -= 4;
  int fctl = ntohs(frameControl->fctl);
  const wifi_ieee80211_packet_t *ipkt = (wifi_ieee80211_packet_t *)snifferPacket->payload;
  const WifiMgmtHdr *hdr = &ipkt->hdr;

  // If we dont the buffer size is not 0, don't write or else we get CORRUPT_HEAP
  if (snifferPacket->payload[0] == 0x80)
  {
    memcpy(bdt.ssid, &snifferPacket->payload[41], SSID_length);
    bdt.ssid[SSID_length] = 0;
    Serial.print("SSID: "); Serial.print(bdt.ssid);
    getMAC(bdt.addr1, snifferPacket->payload, 4);
    Serial.print(" target MAC: "); Serial.print(bdt.addr1);
    getMAC(bdt.addr2, snifferPacket->payload, 10);
    Serial.print(" source MAC: "); Serial.print(bdt.addr2);
    getMAC(bdt.addr3, snifferPacket->payload, 16);
    Serial.print(" filter MAC: "); Serial.print(bdt.addr3);
    Serial.print(" len: ");Serial.print(len, DEC);
    // ID balise
    trame += "ID=";
    Serial.print(" ID: ");  printDataSpan(offset_OUI+4+6, len, TLV_LENGTH[ID_FR] , snifferPacket->payload);
    memcpy(&bdt.ID, &snifferPacket->payload[offset_OUI+4+6], TLV_LENGTH[ID_FR]);
    bdt.ID[30] = '\0';
 
    uint16_t offset = offset_OUI+4+6+TLV_LENGTH[ID_FR]+2; // +2 : Type + Length
    // Latitude
    trame += " lat=";
    Serial.print(" LAT: ");
    bdt.lat = printCoordinates(offset, len, TLV_LENGTH[HOME_LATITUDE] , snifferPacket->payload);
    offset += TLV_LENGTH[LATITUDE]+2;
    // Longitude
    trame += " long=";
    Serial.print(" LON: ");
    bdt.lon = printCoordinates(offset, len, TLV_LENGTH[HOME_LONGITUDE] , snifferPacket->payload);
    offset += TLV_LENGTH[LONGITUDE]+2;
    //Altitude msl
    trame += " alt=";
    bdt.alt = convertToInt(offset, len, TLV_LENGTH[ALTITUDE] , snifferPacket->payload);
    Serial.print(" ALT ABS: "); printAltitude(offset, len, TLV_LENGTH[ALTITUDE] , snifferPacket->payload);
    offset += TLV_LENGTH[ALTITUDE]+2;
    //home altitude
    trame += " Hauteur=";
    bdt.height = convertToInt(offset, len, TLV_LENGTH[HEIGHT] , snifferPacket->payload);
    Serial.print(" HAUTEUR: "); printAltitude(offset, len, TLV_LENGTH[HEIGHT] , snifferPacket->payload);
    offset += TLV_LENGTH[HEIGHT]+2;
    //home latitude
    trame += " lat dep=";
    Serial.print(" LAT DEP: ");
    bdt.dep_lat = printCoordinates(offset, len, TLV_LENGTH[HOME_LATITUDE] , snifferPacket->payload);
    offset += TLV_LENGTH[HOME_LATITUDE]+2;
    //home longitude
    trame += " long dep=";
    Serial.print(" LON DEP: ");
    bdt.dep_lon = printCoordinates(offset, len, TLV_LENGTH[HOME_LONGITUDE] , snifferPacket->payload);
    offset += TLV_LENGTH[HOME_LATITUDE]+2;
    //ground speed
    trame += " Vitesse=";
    bdt.speed = convertToInt(offset, len, TLV_LENGTH[GROUND_SPEED], snifferPacket->payload);
    Serial.print(" VITESSE HOR: "); printAltitude(offset, len, TLV_LENGTH[GROUND_SPEED] , snifferPacket->payload);
    offset += TLV_LENGTH[GROUND_SPEED]+2;
    //heading
    trame += " Dir=";
    bdt.hdg = convertToInt(offset, len, TLV_LENGTH[HEADING] , snifferPacket->payload);
    Serial.print(" DIR: "); printAltitude(offset, len, TLV_LENGTH[HEADING] , snifferPacket->payload);
    trame="";
    Serial.print("RSSI: ");
    Serial.println(snifferPacket->rx_ctrl.rssi);
    bdt.rssi = snifferPacket->rx_ctrl.rssi;
    bdt.ts = millis();
    armDisplay();
  }
}

// the setup function runs once when you press reset or power the board
void setup() {
  setCpuFrequencyMhz(80);
  Serial.begin(115200);
  delay(50);
  Serial.println("FFAM Balise Receiver starting");
  wifi_sniffer_init();
  tft.init();
  tft.fillScreen(TFT_BLACK);
  tft.setRotation(1);
  tft.setTextFont(4);
  bdt.alt = -99.99;
  bdt.speed = 0;
  snprintf(bdt.ID, 31, "!!!!!!ATTENTE  PASDE BALISE !!");
  bdt.ts = millis();
  btn1.setClickHandler([](Button2 & b) {
    page = (page - 1) >= 0 ? page - 1 : NB_PAGES -1;
    must_display = true;
    must_clean_display = true;
  });
  btn2.setClickHandler([](Button2 & b) {
    page = (page + 1) < NB_PAGES ? page + 1 : 0;
    must_display = true;
    must_clean_display = true;
  });
  btn1.setLongClickHandler([](Button2 & b) {
    int r = digitalRead(TFT_BL);
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("Mise en Veille...",  tft.width() / 2, tft.height() / 2, 4);
    delay(2000);
    digitalWrite(TFT_BL, !r);
    tft.writecommand(TFT_DISPOFF);
    tft.writecommand(TFT_SLPIN);
    //After using light sleep, you need to disable timer wake, because here use external IO port to wake up
    //esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
    // esp_sleep_enable_ext1_wakeup(GPIO_SEL_35, ESP_EXT1_WAKEUP_ALL_LOW);
    //esp_sleep_enable_ext0_wakeup(GPIO_NUM_35, 0);
    //delay(200);
    digitalWrite(ADC_EN, LOW);
    digitalWrite(ADC_PIN, LOW);
    digitalWrite(TFT_CS, LOW);
    digitalWrite(TFT_DC, LOW);
    digitalWrite(TFT_RST, LOW);
    digitalWrite(TFT_MOSI, LOW);
    digitalWrite(TFT_MISO, LOW);
    rtc_gpio_isolate((gpio_num_t) ADC_EN);
    rtc_gpio_isolate((gpio_num_t) ADC_PIN);
    rtc_gpio_isolate((gpio_num_t) BUTTON_1);
    rtc_gpio_isolate((gpio_num_t) BUTTON_2);
    rtc_gpio_isolate((gpio_num_t) TFT_CS);
    rtc_gpio_isolate((gpio_num_t) TFT_DC);
    rtc_gpio_isolate((gpio_num_t) TFT_RST);
    rtc_gpio_isolate((gpio_num_t) TFT_BL);
    rtc_gpio_isolate((gpio_num_t) TFT_MOSI);
    rtc_gpio_isolate((gpio_num_t) TFT_MISO);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
    esp_deep_sleep_start();
  });
  esp_adc_cal_characteristics_t adc_chars;
  esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);    //Check type of calibration value used to characterize ADC
  if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
    vref = adc_chars.vref;
  }
  pinMode(ADC_EN, OUTPUT); 
  digitalWrite(ADC_EN, HIGH);
  battUpdate();
}

void armDisplay() {
  must_display = true;
}

void battUpdate() {
  adc_power_acquire();
  uint16_t v = analogRead(ADC_PIN);
  adc_power_release();
  battery_voltage = ((float)v / 4095.0) * 2.0 * 3.3 * (vref / 1000.0);
  battTicker.once(5, battUpdate);
}

void updateDisplay() {
  char st[64];
  uint16_t y_offset = 37;
  uint8_t char_width = 14;
  if (must_display) {
    must_display = false;
    if (must_clean_display) {
      must_clean_display = false;
      tft.fillScreen(TFT_BLACK);
      tft.setTextDatum(BL_DATUM);
      tft.setTextColor(TFT_CYAN, TFT_BLACK);
      tft.drawString("Info Balises v0.1", 0, 135, 2);
      sprintf(st, "%d/%d", page + 1, NB_PAGES);
      tft.setTextDatum(BR_DATUM);
      tft.drawString(st, 239, 135, 2);
      tft.drawFastHLine(0, 117, 239, TFT_CYAN);
      tft.drawFastVLine(210, 117, 17, TFT_CYAN);
      tft.drawFastVLine(162, 117, 17, TFT_CYAN);
      tft.drawFastVLine(113, 117, 17, TFT_CYAN);
      tft.setTextDatum(L_BASELINE);
      tft.setFreeFont(FSS12);
      tft.setTextColor(TFT_YELLOW, TFT_BLACK);
      tft.drawString("ID", 8, 22);
      tft.drawFastHLine(0, 32, 239, TFT_YELLOW);
      tft.drawFastVLine(42, 0, 32, TFT_YELLOW);
      last_frame_ts = 0;
      last_displayed_batt_voltage = 0.0;
      last_displayed_page = NB_PAGES;
    }
    uint16_t age_in_s = (millis() - bdt.ts) / 1000;
    if (age_in_s <= 3) {
      tft.setTextColor(TFT_CYAN, TFT_BLACK);
    } else if (age_in_s <= 6) {
      tft.setTextColor(TFT_ORANGE, TFT_BLACK);
    } else {
      tft.setTextColor(TFT_RED, TFT_BLACK);
    }
    tft.setTextDatum(BR_DATUM);
    if (age_in_s < 120) {
      sprintf(st, "%4ds", age_in_s);
    } else if (age_in_s < 120 * 60) {
      sprintf(st, "%4dm", age_in_s / 60);
    } else {
      sprintf(st, "%4dh", age_in_s / 3600);
    }
    tft.drawString(st, 157, 135, 2);
    if (battery_voltage != last_displayed_batt_voltage) {
      sprintf(st, "%.2fV", battery_voltage);
      tft.setTextDatum(BR_DATUM);
      if (battery_voltage > 3.8) {
        tft.setTextColor(TFT_CYAN, TFT_BLACK);
      } else if (battery_voltage > 3.7) {
        tft.setTextColor(TFT_ORANGE, TFT_BLACK);
      } else {
        tft.setTextColor(TFT_RED, TFT_BLACK);
      }
      tft.drawString(st, 205, 135, 2);
      last_displayed_batt_voltage = battery_voltage;
    }
    if (last_frame_ts != bdt.ts) {
      last_frame_ts = bdt.ts;
      char builder[4]="XXX", version[4]="NNN", id_start[9] = "00000000", id_gmmm[5] = "GMMM";
      strncpy(builder, bdt.ID, 3);
      strncpy(version, &bdt.ID[3], 3);
      strncpy(id_start, &bdt.ID[6], 8);
      strncpy(id_gmmm, &bdt.ID[14], 4);
      tft.setTextDatum(R_BASELINE);
      tft.setFreeFont(FM9);
      tft.setTextColor(TFT_YELLOW, TFT_BLACK);
      sprintf(st, "%s %s  %s", builder, version, id_start);
      tft.drawString(st, 235, 10);
      sprintf(st, "%s %s", id_gmmm, &bdt.ID[18]);
      tft.drawString(st, 235, 26);
      switch (page) {
        case 0:
          if (last_displayed_page != page) {
            tft.setTextColor(TFT_WHITE, TFT_BLACK);
            tft.setTextDatum(L_BASELINE);
            tft.setFreeFont(FF17);
            tft.drawString("Alt.", 0, 30 + y_offset);
            tft.drawString("Haut.", 0, 70 + y_offset);
            tft.drawString("Route", 129,  30 + y_offset);
            tft.drawString("Vit.", 129, 70 + y_offset);
            tft.setTextDatum(R_BASELINE);
            tft.drawString("m", 120, 30  + y_offset, 2);
            tft.drawString("m", 120, 70 +  + y_offset, 2);
            tft.drawString("m/s", 239, 70 + y_offset, 2);
          }
          tft.setFreeFont(FMB18);
          tft.setTextColor(TFT_GREEN, TFT_BLACK);
          tft.setTextDatum(R_BASELINE);
          sprintf(st, "%4d", bdt.alt);
          tft.drawString(st, 120 - char_width, y_offset + 30);
          sprintf(st, "%3d", bdt.height);
          tft.drawString(st, 120 - char_width, y_offset + 70);
          sprintf(st, "%3d", bdt.hdg);
          tft.drawString(st, 239, y_offset + 30);
          sprintf(st, "%2d", bdt.speed);
          tft.drawString(st, 239 - (char_width * 2), y_offset + 70);
          tft.drawFastHLine(0, 75, 239, TFT_WHITE);
          tft.drawFastVLine(123, 32, 85, TFT_WHITE);
          break;
        case 1:
          if (last_displayed_page != page) {
            tft.setTextColor(TFT_WHITE, TFT_BLACK);
            tft.setTextDatum(L_BASELINE);
            tft.setFreeFont(FF17);
            tft.drawString("Latitude. vol", 0, 16 + y_offset);
            tft.drawString("Longitude vol", 0, 35 + y_offset);
            tft.drawString("Latitude dep.", 0, 54 + y_offset);
            tft.drawString("Longitude dep.", 0, 73 + y_offset);
          }
          tft.setTextColor(TFT_GREEN, TFT_BLACK);
          tft.setFreeFont(FM9);
          tft.setTextDatum(R_BASELINE);
          sprintf(st, "%.4f %c", abs(bdt.lat), bdt.lat < 0.0 ? 'S': 'N');
          tft.drawString(st, 239, 16 + y_offset);
          sprintf(st,  "%3.4f %c", abs(bdt.lon), bdt.lat < 0.0 ? 'W': 'E');
          tft.drawString(st, 239, 35 + y_offset);
          sprintf(st, "%2.4f %c", abs(bdt.dep_lat), bdt.dep_lat < 0.0 ? 'S': 'N');
          tft.drawString(st, 239, 54 + y_offset);
          sprintf(st, "%3.4f %c", abs(bdt.dep_lon), bdt.dep_lon < 0.0 ? 'W' : 'E');
          tft.drawString(st, 239, 73 + y_offset);
          break;
        case 2:
          if (last_displayed_page != page) {
            tft.setTextColor(TFT_WHITE, TFT_BLACK);
            tft.setTextDatum(L_BASELINE);
            tft.setFreeFont(FF17);
            tft.drawString("RSSI", 0, 18 + y_offset);
            tft.drawString("MAC", 0, 42 + y_offset);
            tft.drawString("SSID", 0, 66 + y_offset);
          }
          tft.setTextColor(TFT_GREEN, TFT_BLACK);
          tft.setFreeFont(FM9);
          tft.setTextDatum(R_BASELINE);
          sprintf(st, "%4d", bdt.rssi);
          tft.drawString(st, 239, 18 + y_offset);
          tft.drawString(bdt.addr2, 239, 42 + y_offset);
          tft.drawString(bdt.ssid, 239, 66 + y_offset);
      }
      if (last_displayed_page != page) {
        last_displayed_page = page;
      }
    }
    displayTicker.once(1, armDisplay);
  }
}

// the loop function runs over and over again forever
void loop() {
  btn1.loop();
  btn2.loop();
  updateDisplay();
  delay(50);
}
