#ifndef NMEA_PARSER_H
#define NMEA_PARSER_H

#include "nmea_sent.h"

#define NMEA_PARSER_BUFFER_SIZE 1024

typedef struct _nmea_parser_t {
  char* buffer;
  int buffer_size;
  int buffer_used;
  void (*show_sentence)(nmea_gprmc_t *info);
} nmea_parser_t;

nmea_parser_t *nmea_parser_init(void);
nmea_parser_t *get_nmea_parser(void);

int nmea_parser_parse(nmea_parser_t* parser, const char *data, int data_size);

int nmea_parser_buffer_clear(nmea_parser_t* parser);

#endif