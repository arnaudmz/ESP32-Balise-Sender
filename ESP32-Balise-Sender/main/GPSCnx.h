// vim:et:sts=2:sw=2:si
#ifndef __GPSCnx_h
#define __GPSCnx_h

#include "driver/uart.h"
#include "driver/i2c.h"
#include "TinyGPS++.h"
#define RX_BUF_SIZE 1024

class GPSCnx {
  public:
    GPSCnx(TinyGPSPlus *gps): gps(gps) {}
    void begin();
    uint32_t wait_for_chars(int first_timeout_ms = 1000, int next_timeout_ms = 20, bool inject = true);
  private:
    TinyGPSPlus *gps;
    uint8_t rx_buffer[RX_BUF_SIZE];
#ifdef CONFIG_BEACON_GPS_MOCK
    char mock_msg[256];
    double mock_gps_home_lat = 4843.20138;
    double mock_gps_home_long = 211.47344;
    double mock_gps_home_ts = 145203;
    double mock_gps_raduis = 0.30;
    int mock_gps_home_alt = 148;
    int mock_gps_max_height = 150;
    int mock_gps_min_height = 10;
    void compute_mock_msg();
#endif //#ifdef CONFIG_BEACON_GPS_MOCK
#ifdef CONFIG_BEACON_GPS_L96_I2C
    const i2c_port_t i2c_port  = I2C_NUM_0;
    uint8_t i2c_buf[I2C_BUF_SIZE];
    void i2c_setup();
#endif //ifdef CONFIG_BEACON_GPS_L96_I2C
#if defined(CONFIG_BEACON_GPS_L96_UART) || defined(CONFIG_BEACON_GPS_L80R_UART) || defined(CONFIG_BEACON_GPS_BN_220_UART)
    const uart_port_t uart_port = UART_NUM_2;
    void uart_setup();
    void uart_send_PMTK(const char *st);
    void uart_wait_for_silence(int timeout_ms = 30);
#endif //#if defined(CONFIG_BEACON_GPS_L96_UART) || defined(CONFIG_BEACON_GPS_L80R_UART) || defined(CONFIG_BEACON_GPS_BN_220_UART)
    uint8_t compute_PMTK_cksum(const char *st);
};

#endif //ifndef __GPSCnx_h
