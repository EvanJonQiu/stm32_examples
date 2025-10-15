#include "nmea_sent.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

int nmea_sent_type(char* sentence, int sentence_length) {

  if (sentence_length < 6) {
    return -1;
  } else if (0 == memcmp(sentence, "$GNRMC", 6)) {
    return GNRMC;
  } else {
    return NONE;
  }
}

int nmea_sent_parse_gprmc(char* sentence, int sentence_length, nmea_gprmc_t* gprmc) {

  char *pos = 0;
  int index = -1;
  char *field_begin = 0;
  char *field_end = 0;
  int res_crc = -1;
  int crc = 0;

  if (sentence_length < 6) {
    return -1;
  }

  for (pos = sentence + 1; pos < sentence + sentence_length; pos++) {
    if (*pos != '*') {
      crc ^= *pos;
      continue;
    }
    res_crc = strtol(pos + 1, NULL, 16);
    if (crc != res_crc) {
      return -1;
    }
    break;
  }

  if (res_crc == -1) {
    return -1;
  }

  while ((pos = strchr(sentence, ',')) != NULL) {
    index++;

    if (index == 1) {
      if (pos[1] == 'A') {
        gprmc->status = 1;
      } else {
        gprmc->status = 0;
      }
    } else if (index == 2) {
      field_begin = pos + 1;
    } else if (index == 3) {
      if (pos[1] == 'N' || pos[1] == 'S') {
        gprmc->uLat = pos[1];
      } else {
        gprmc->uLat = 'N';
      }
      field_end = pos;
      int field_length = field_end - field_begin;
      memset(gprmc->lat, 0, sizeof(gprmc->lat));
      memcpy(gprmc->lat, field_begin, field_length);
    } else if (index == 4) {
      field_begin = pos + 1;
    } else if (index == 5) {
      if (pos[1] == 'E' || pos[1] == 'W') {
        gprmc->uLon = pos[1];
      } else {
        gprmc->uLon = 'E';
      }
      field_end = pos;
      int field_length = field_end - field_begin;
      memset(gprmc->lon, 0, sizeof(gprmc->lon));
      memcpy(gprmc->lon, field_begin, field_length);
    }
    sentence = pos + 1;
  }

  return 0;
}