// vim:et:sts=2:sw=2:si
#include <cmath>

uint8_t compute_PMTK_cksum(const char *st);
extern bool has_set_home;

char mock_msg[256];
double mock_gps_home_lat = 4843.20138;
double mock_gps_home_long = 211.47344;
double mock_gps_home_ts = 145203;
double mock_gps_raduis = 0.30;
int mock_gps_home_alt = 334;
int mock_gps_max_height = 150;
int mock_gps_min_height = 10;
uint8_t mock_cursor = 0;

void compute_mock_msg() {
  double lat;
  double longi;
  double heading;
  double speed_in_knots;
  int alt;
  char raw_gprmc_msg[120], raw_gpgga_msg[120];
  int gprmc_cksum, gpgga_cksum;
  if (!has_set_home) {
    lat = mock_gps_home_lat;
    longi = mock_gps_home_long;
    heading = 0;
    speed_in_knots = 0.0;
    alt = mock_gps_home_alt;
  } else {
    int secs_from_start = millis() / 1000;
    double pos_angle = (double)secs_from_start * M_TWOPI / 66.33;
    double alt_angle = (double)secs_from_start * M_TWOPI / 92.35;
    ESP_LOGV(TAG, "Computed pos_angle: %f", pos_angle);
    ESP_LOGV(TAG, "Computed alt_angle: %f", alt_angle);
    lat = mock_gps_home_lat + mock_gps_raduis * sin(pos_angle);
    longi = mock_gps_home_long + mock_gps_raduis * cos(pos_angle);
    heading = fmod(360 - fmod(pos_angle * 360.0 / M_TWOPI, 360), 360);
    speed_in_knots = 20.0 + 10.0 * sin(pos_angle);
    alt = mock_gps_home_alt + (mock_gps_max_height + mock_gps_min_height) / 2 + (mock_gps_max_height -  mock_gps_min_height) / 2 * cos(alt_angle);
  }
  ESP_LOGV(TAG, "Computed heading: %f", heading);
  ESP_LOGV(TAG, "Computed speed: %f", speed_in_knots);
  ESP_LOGV(TAG, "Computed alt: %d", alt);
  ESP_LOGV(TAG, "Computed lat: %f", lat);
  ESP_LOGV(TAG, "Computed longi: %f", longi);
  snprintf(raw_gprmc_msg, 128, "GPRMC,015606.000,A,%.4f,N,%4f,E,%.2f,%.2f,280715,,,A",
      lat,
      longi,
      speed_in_knots,
      heading);
  gprmc_cksum = compute_PMTK_cksum(raw_gprmc_msg);
  snprintf(raw_gpgga_msg, 128, "GPGGA,015606.000,%.4f,N,%.4f,E,1,7,1.28,%d.0,M,0.0,M,,",
      lat,
      longi,
      alt);
  gpgga_cksum = compute_PMTK_cksum(raw_gpgga_msg);
  snprintf(mock_msg, 256, "$%s*%2X\r\n$%s*%2X\r\n", raw_gprmc_msg, gprmc_cksum, raw_gpgga_msg, gpgga_cksum);
  ESP_LOGV(TAG, "msg: %s", mock_msg);
}
