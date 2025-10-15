#include "nmea_parser.h"


#include <string.h>
#include <stdio.h>

static char _nmea_parser_buffer[1024];

static nmea_parser_t _nmea_parser;

static nmea_gprmc_t gprmc_info;

nmea_parser_t* nmea_parser_init(void) {

  memset(&_nmea_parser, 0, sizeof(_nmea_parser));
  _nmea_parser.buffer = _nmea_parser_buffer; 
  _nmea_parser.buffer_size = NMEA_PARSER_BUFFER_SIZE;
  _nmea_parser.buffer_used = 0;

  memset(&gprmc_info, 0, sizeof(gprmc_info));

  return &_nmea_parser;
}

nmea_parser_t* get_nmea_parser(void) {
  return &_nmea_parser;
}

int nmea_parser_parse(nmea_parser_t* parser, const char *data, int data_size) {

  char *pos = 0;
  char *buffer_end = 0;
  char *sentence_begin = 0;
  int parsed_length = 0;

  if (parser->buffer_used + data_size >= parser->buffer_size) {
    nmea_parser_buffer_clear(parser);
  }

  memcpy(parser->buffer + parser->buffer_used, data, data_size);
  parser->buffer_used += data_size;

  pos = parser->buffer;
  buffer_end = parser->buffer + parser->buffer_used;

  while (pos < buffer_end) {
    if (*pos == '$' && !sentence_begin) {
      sentence_begin = pos;
      pos++;
      continue;
    } else if (*pos == '\n') {
      if (sentence_begin) {
        pos++;
        int sentence_length = pos - sentence_begin;
        int sent_type = nmea_sent_type(sentence_begin, sentence_length);
        if (sent_type == GNRMC) {
          memset(&gprmc_info, 0, sizeof(gprmc_info));
          if (nmea_sent_parse_gprmc(sentence_begin, sentence_length, &gprmc_info) == 0) {
            if (parser->show_sentence) {
              parser->show_sentence(&gprmc_info);
            }
          }
        }
        parsed_length += sentence_length;
      }
      sentence_begin = 0;
    }
    pos++;
  }

  if (parser->buffer_used - parsed_length > 0) {
    memmove(parser->buffer, parser->buffer + parsed_length, parser->buffer_used - parsed_length);
    parser->buffer_used -= parsed_length;
  } else {
    parser->buffer_used = 0;
  }
  return 0;
}

int nmea_parser_buffer_clear(nmea_parser_t* parser) {
  memset(parser->buffer, 0, parser->buffer_size);
  parser->buffer_used = 0;
  return 0;
}