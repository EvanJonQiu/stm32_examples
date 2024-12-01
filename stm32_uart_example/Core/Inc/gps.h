/*
 * gps.h
 *
 */

#ifndef INC_GPS_H_
#define INC_GPS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "usart.h"

extern char rx_buf[5];
extern char response_buf[256];

void init_screen();
void show_counter();

void send_cmd(char *cmd, uint8_t size, char *type);

#ifdef __cplusplus
}
#endif

#endif /* INC_GPS_H_ */
