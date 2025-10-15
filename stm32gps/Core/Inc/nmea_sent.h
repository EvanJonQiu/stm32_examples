#ifndef NMEA_SENT_H
#define NMEA_SENT_H

enum nmea_sent_type {
  NONE = 0x0000,
  GNRMC = 0x0001
};

typedef struct _nmea_gprmc_t {
  char lat[16];
  char uLat;
  char lon[16];
  char uLon;
  int  status;
} nmea_gprmc_t;

int nmea_sent_type(char* sentence, int sentence_length);

int nmea_sent_parse_gprmc(char* sentence, int sentence_length, nmea_gprmc_t* gprmc);

#endif