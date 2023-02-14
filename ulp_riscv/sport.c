// vim:et:sts=2:sw=2:si

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
//#include "ulp_riscv_register_ops.h"
#include "ulp_riscv_utils.h"
#include "ulp_riscv_gpio.h"

#include "ulp_shared_types.h"
#include "dbg.h"
#include "uart.h"
#include "sport.h"

#define SPORT_PROLOG0             0x7e
#define SPORT_SENSOR_ID_GPS       0xf2 // 0x12 with CRC aka ID19
#define SPORT_SENSOR_ID_GPS_TX    0x53 // 0x13 with CRC aka ID20

#define SPORT_GPS_METRIC_LAT_LON_ID   0x0800
#define SPORT_GPS_METRIC_ALT_ID       0x0820
#define SPORT_GPS_METRIC_SPEED_ID     0x0830
#define SPORT_GPS_METRIC_COG_ID       0x0840
#define SPORT_GPS_METRIC_TS_ID        0x0850
#define SPORT_BEACON_METRIC_STAT_ID   0x5200
#define SPORT_BEACON_METRIC_SAT_ID    0x5210
#define SPORT_BEACON_METRIC_HDOP_ID   0x5220
#define SPORT_BEACON_METRIC_PREFIX_ID 0x5230
#define SPORT_BEACON_METRIC_FRAMES_ID 0x5240

typedef struct {
  uint16_t id;
  uint32_t period_ms;
  uint32_t last_sent_ts;
  volatile uint32_t *value;
} SPortMetric;

/* these variables will be exported as public symbols, visible from main CPU */
volatile uint32_t millis;
volatile uint32_t metric_gps_lat;
volatile uint32_t metric_gps_lon;
volatile uint32_t metric_gps_alt;
volatile uint32_t metric_gps_speed;
volatile uint32_t metric_gps_cog;
volatile uint32_t metric_gps_date;
volatile uint32_t metric_gps_time;
volatile uint32_t metric_beacon_stat;
volatile uint32_t metric_beacon_sat;
volatile uint32_t metric_beacon_hdop;
volatile uint32_t metric_beacon_prefix;
volatile uint32_t metric_beacon_frames;

volatile uint32_t metric_beacon_cmd_pending = 0;
volatile uint32_t metric_beacon_cmd = 0;
volatile uint32_t metric_beacon_new_prefix;


enum {
  SPORT_METRIC_GPS_LAT = 0,
  SPORT_METRIC_GPS_LON,
  SPORT_METRIC_GPS_ALT,
  SPORT_METRIC_GPS_SPEED,
  SPORT_METRIC_GPS_COG,
  SPORT_METRIC_GPS_DATE,
  SPORT_METRIC_GPS_TIME,
  SPORT_METRIC_BEACON_STAT,
  SPORT_METRIC_BEACON_SAT,
  SPORT_METRIC_BEACON_HDOP,
  SPORT_METRIC_BEACON_PREFIX,
  SPORT_METRIC_BEACON_FRAMES,
  SPORT_METRIC_GPS_LAST
};


static inline void sport_send_encoded_byte(uint8_t b) {
    if(b == 0x7e) {
      uart_send_byte(0x7d);
      uart_send_byte(0x5e);
    } else if (b == 0x7d){
      uart_send_byte(0x7d);
      uart_send_byte(0x5d);
    } else {
      uart_send_byte(b);
    }
}

static uint8_t sport_cksum(const uint8_t *data, uint8_t len) {
  uint32_t total = 0;
  for(int i = 0; i < len; i++) {
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

static inline void sport_send_payload(const uint8_t *data, uint8_t len) {
  uart_prepare_for_tx();
  for (uint8_t i = 0; i < len; i++) {
    sport_send_encoded_byte(data[i]);
  }
  uart_prepare_for_rx();
}

static inline void sport_prepare_too_soon_frame(uint8_t *data) {
  static const uint8_t too_soon[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff};
  memcpy(data, too_soon, 8);
}

static inline void sport_prepare_metric_frame(SPortMetric *m, uint8_t *data) {
  data[0] = 0x10;
  memcpy(&data[1], &(m->id), sizeof(m->id));
  memcpy(&data[3], (void *) m->value, 4);
  data[7] = sport_cksum(data, 7);
  m->last_sent_ts = millis;
}

static inline uint8_t sport_get_next_metric_index(SPortMetric *metrics, uint8_t metric_index) {
  uint8_t i = (metric_index + 1) % SPORT_METRIC_GPS_LAST;
  while (i != metric_index) {
    if (millis - metrics[i].last_sent_ts > metrics[i].period_ms) {
      return i;
    }
    i = (i + 1) % SPORT_METRIC_GPS_LAST;
  }
  return SPORT_METRIC_GPS_LAST;
}

static inline void sport_parse_cmd(uint8_t *data, uint8_t len) {
  uint8_t ck = sport_cksum(data, len - 1);
  if (ck == data[len - 1]) {
    if(data[1] == 0xff && data[2] == 0x00) {
      memcpy((void *) &metric_beacon_new_prefix, &data[3], sizeof(metric_beacon_new_prefix));
      metric_beacon_cmd = SPORT_CMD_NEW_PREFIX;
    } else {
      metric_beacon_cmd = 5;
    }
    metric_beacon_cmd_pending = 1;
  }
}

void sport_handle(uint8_t *data, uint8_t len) {
  static uint8_t response_data[8];
  static SPortMetric metrics[SPORT_METRIC_GPS_LAST] = {
    { SPORT_GPS_METRIC_LAT_LON_ID,    1000, 0, &metric_gps_lat       },
    { SPORT_GPS_METRIC_LAT_LON_ID,    1000, 0, &metric_gps_lon       },
    { SPORT_GPS_METRIC_ALT_ID,        1000, 0, &metric_gps_alt       },
    { SPORT_GPS_METRIC_SPEED_ID,      1000, 0, &metric_gps_speed     },
    { SPORT_GPS_METRIC_COG_ID,        1000, 0, &metric_gps_cog       },
    { SPORT_GPS_METRIC_TS_ID,        10000, 0, &metric_gps_date      },
    { SPORT_GPS_METRIC_TS_ID,         1000, 0, &metric_gps_time      },
    { SPORT_BEACON_METRIC_STAT_ID,    1000, 0, &metric_beacon_stat   },
    { SPORT_BEACON_METRIC_SAT_ID,     1000, 0, &metric_beacon_sat    },
    { SPORT_BEACON_METRIC_HDOP_ID,    1000, 0, &metric_beacon_hdop   },
    { SPORT_BEACON_METRIC_PREFIX_ID,  1000, 0, &metric_beacon_prefix },
    { SPORT_BEACON_METRIC_FRAMES_ID,  5000, 0, &metric_beacon_frames }
  };
  static uint8_t metric_index = SPORT_METRIC_GPS_LAST - 1;
  static uint8_t delay_loop = 0;
  static bool delay_armed = false;
  if (len == 2 && data[0] == SPORT_PROLOG0 && data[1] == SPORT_SENSOR_ID_GPS){
    // regular query to get on of my metrics !
    uint8_t next_metric_index = sport_get_next_metric_index(metrics, metric_index);
    if (next_metric_index == SPORT_METRIC_GPS_LAST) {
      sport_prepare_too_soon_frame(response_data);
    } else {
      metric_index = next_metric_index;
      sport_prepare_metric_frame(&metrics[metric_index], response_data);
    }
    delay_armed = true;
    delay_loop = 0;
  } else if (len == 10 && data[0] == SPORT_PROLOG0 && data[1] == SPORT_SENSOR_ID_GPS_TX) {
    // a command from TX to me
    // so far, only changing prefix is implemented
    sport_parse_cmd(&data[2], len - 2);
  } else if (delay_armed) {
    if (delay_loop == 25) {
      sport_send_payload(response_data, 8);
      delay_armed = false;
    } else {
      delay_loop ++;
    }
  }
}
