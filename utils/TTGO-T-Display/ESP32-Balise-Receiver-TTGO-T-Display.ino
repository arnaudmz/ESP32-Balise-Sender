// vim:et:sts=2:sw=2:si
#include <byteswap.h>
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
#define OUI_VER 0x01355C6A

#define B_VERSION 0x01
#define B_IDENT_FR 0x02
#define B_IDENT_ANSI 0x03
#define B_CUR_LAT 0x04
#define B_CUR_LON 0x05
#define B_CUR_ALT 0x06
#define B_CUR_HAU 0x07
#define B_STA_LAT 0x08
#define B_STA_LON 0x09
#define B_CUR_VIT 0x0A
#define B_CUR_DIR 0x0B

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

typedef struct {
  uint8_t type;       /**<  */
  uint8_t length;       /**<  */
  uint8_t payload[];      /**<  */
} __attribute((__packed__)) TLV_t;

typedef struct {
  uint8_t subtype;              /**<  Frame subtype*/
  uint8_t type;               /**<  Frame typa*/
  uint16_t duration;              /**<  */
  uint8_t receiver_address[6];        /**<  Receiver mac-address*/
  uint8_t transmitter_address[6];       /**<  Transmitter mac-address*/
  uint8_t BSS_Id[6];              /**<  */
  uint16_t sequence_frag;           /**< 4 bits LSB frag, 12 bits MSB sequence sequence_frag = (0x000F & (frag)) | ( 0x0FFF & sequence<<4) */
  uint64_t timestamp;             /**<  Timestamp of frame*/
  uint16_t beacon_interval;         /** Interval between beacon frame */
  uint16_t capability;            /**mandatory fields */  
  uint8_t ssid_element_id;          /** SSID Element ID */
  uint8_t ssid_length;            /** SSID length */
  uint8_t ssid_value[]; /** SSID value*/

} __attribute((__packed__)) frame_header_t;

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
  char addr[19];
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


void getMAC(char *addr, uint8_t* data, uint16_t offset) {
  sprintf(addr, "%02X:%02X:%02X:%02X:%02X:%02X", data[offset+0], data[offset+1], data[offset+2], data[offset+3], data[offset+4], data[offset+5]);
}

static void beaconCallback(void *recv_buf, wifi_promiscuous_pkt_type_t type) {

  static int32_t *oui_ver;
  static int32_t index, ptr, i;
  static int16_t i_value;
  static uint8_t *hex;
  static int8_t s_value;

  static wifi_promiscuous_pkt_t *sniffer;
  static TLV_t *tlv, *b_tlv;
  static vendor_ie_data_t *gse_vendor;
  static frame_header_t *packet;
  static float f_value;

  sniffer = (wifi_promiscuous_pkt_t *)recv_buf;

  if ((*(uint16_t *)sniffer->payload) == 0x0080) //beacon frame
  {
    packet = (frame_header_t*) sniffer->payload;
    index = sizeof(*packet) + packet->ssid_length;
    while (index < (sniffer->rx_ctrl.sig_len - 4))
    {
      tlv = (TLV_t*)((sniffer->payload) + index);
      //WIFI_VENDOR_IE_ELEMENT_ID  ???
      if (tlv->type == 0xDD)
      {
        oui_ver = (int32_t*) tlv->payload;
        if ( *oui_ver == OUI_VER) {
          gse_vendor = (vendor_ie_data_t*)(tlv);
          ptr = 0;
          while (ptr < (gse_vendor->length - 4))
          {
            b_tlv = (TLV_t*)((gse_vendor->payload) + ptr);
            switch (b_tlv->type)
            {
              case B_IDENT_FR :
                memcpy(bdt.ID, b_tlv->payload, b_tlv->length);
                bdt.ID[b_tlv->length + 1] = '\0';
                break;
              case B_CUR_LAT :
                f_value = (float)((signed)(__bswap_32(*(int32_t*)(b_tlv->payload)))) / 100000.0;
                bdt.lat = f_value;
                break;
              case B_CUR_LON :
                f_value = (float)((signed)(__bswap_32(*(int32_t*)(b_tlv->payload)))) / 100000.0;
                bdt.lon = f_value;
                break;
              case B_CUR_ALT :
                i_value = __bswap_16(*(int16_t*)(b_tlv->payload));
                bdt.alt = i_value;
                break;
              case B_CUR_HAU :
                i_value = __bswap_16(*(int16_t*)(b_tlv->payload));
                bdt.height = i_value;
                break;
              case B_STA_LAT :
                f_value = (float)((signed)(__bswap_32(*(int32_t*)(b_tlv->payload)))) / 100000.0;
                bdt.dep_lat = f_value;
                break;
              case B_STA_LON :
                f_value = (float)((signed)(__bswap_32(*(int32_t*)(b_tlv->payload)))) / 100000.0;
                bdt.dep_lon = f_value;
                break;
              case B_CUR_VIT :
                s_value = *(b_tlv->payload);
                bdt.speed = s_value;
                break;
              case B_CUR_DIR :
                i_value = __bswap_16(*(int16_t*)(b_tlv->payload));
                bdt.hdg = i_value;
                break;
              default:
                break;
            }
            ptr = ptr + (b_tlv->length) + 2;
          }

          bdt.rssi = sniffer->rx_ctrl.rssi;
          getMAC(bdt.addr, packet->transmitter_address, 0);
          ////construire la trame dans l'ordre:
          trame = "{\"id\": \"";
          trame += bdt.ID;
          trame += "\", \"lat\": ";
          trame += String(bdt.lat, 5);
          trame += ", \"long\": ";
          trame += String(bdt.lon, 5);
          trame += ", \"alt\": ";
          trame += bdt.alt;
          trame += ", \"height\": ";
          trame += bdt.height;
          trame += ", \"dep_lat\": ";
          trame += String(bdt.dep_lat, 5);
          trame += ", \"dep_long\": ";
          trame += String(bdt.dep_lon, 5);
          trame += ", \"speed\": ";
          trame += bdt.speed;
          trame += ", \"heading\": ";
          trame += bdt.hdg;
          trame += ", \"rssi\": ";
          trame += String(bdt.rssi);
          trame += ", \"source_mac\": \"";
          trame += String(bdt.addr);
          trame += "\"}";
          Serial.println(trame);
          trame = "";
          bdt.ts = millis();
          armDisplay();
        }
      }
      index = index + tlv->length + 2;
    }
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
  esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
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
      tft.drawString("Info Balises v0.3", 0, 135, 2);
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
          sprintf(st,  "%3.4f %c", abs(bdt.lon), bdt.lon < 0.0 ? 'W': 'E');
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
          }
          tft.setTextColor(TFT_GREEN, TFT_BLACK);
          tft.setFreeFont(FM9);
          tft.setTextDatum(R_BASELINE);
          sprintf(st, "%4d", bdt.rssi);
          tft.drawString(st, 239, 18 + y_offset);
          tft.drawString(bdt.addr, 239, 42 + y_offset);
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
