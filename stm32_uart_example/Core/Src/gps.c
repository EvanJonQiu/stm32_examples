/*
 * gps.c
 *
 */
#include "gps.h"

#include "ssd1306.h"

#include <string.h>

char string[5] = {0};

char rx_buf[5] = {0};
char response_buf[256] = {0};

void init_screen() {
  SSD1306_GotoXY (0,0);
  SSD1306_Puts ("SSD1306", &Font_11x18, 1);
  SSD1306_GotoXY (0, 30);
  SSD1306_Puts ("OLED DEMO", &Font_11x18, 1);
  SSD1306_UpdateScreen();
  HAL_Delay (1000);

  SSD1306_ScrollRight(0,5);
  HAL_Delay(3000);
  SSD1306_ScrollLeft(0,5);
  HAL_Delay(3000);
  SSD1306_Stopscroll();
  SSD1306_Clear();
  SSD1306_GotoXY (30,0);
  SSD1306_Puts ("COUNTER", &Font_11x18, 1);
}

void show_counter() {
  for (int num = 1;num <= 1000;num++)
  {
    itoa(num, string, 10);
    SSD1306_GotoXY (0, 30);
    SSD1306_Puts ("             ", &Font_16x26, 1);
    SSD1306_UpdateScreen();
    if (num < 10) {
	  SSD1306_GotoXY (53, 30);  // 1 DIGIT
    }
    else if (num < 100) {
	  SSD1306_GotoXY (45, 30);  // 2 DIGITS
    }
    else  {
	  SSD1306_GotoXY (37, 30);  // 3 DIGITS
    }
    SSD1306_Puts (string, &Font_16x26, 1);
    SSD1306_UpdateScreen();
    HAL_Delay (500);
  }
}

void send_cmd(char *cmd, uint8_t size, char *type) {
  HAL_StatusTypeDef status;

  status = HAL_UART_Transmit(&huart2, (uint8_t *)cmd, size, 100);
  if (status == HAL_OK) {
	SSD1306_GotoXY (0, 30);
	SSD1306_Puts (type, &Font_7x10, 1);
	SSD1306_UpdateScreen();
  } else {
    SSD1306_GotoXY (0,30);
    SSD1306_Puts (type, &Font_7x10, 1);
    SSD1306_GotoXY (0, 50);
    SSD1306_Puts ("failure", &Font_7x10, 1);
    SSD1306_UpdateScreen();
  }
}
