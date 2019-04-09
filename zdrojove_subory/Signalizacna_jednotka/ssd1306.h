/*
 * ssd1306.c
 *
 * Created: 03-01-2018 17:33:26
 * Author: osc - edited by Tomas Klein 
 * Source: https://github.com/elo1957/OLED-ssd1306-atmega
 * Description: This modules purpose is to initiate and control the OLED display SSD1306 driver for the 128x64 OLED display 
 *              The plus can be connected to PIN 24 and the GND to PIN 26 and sda goes to PIN 21 and the sck goes to PIN 20
 *				http://microcontrolandos.blogspot.dk/2014/12/pic-ssd1306.html
 */


#ifndef SSD1306_H_
#define SSD1306_H_

#include <stdint.h>

#define SSD1306_LCDWIDTH      128
#define SSD1306_LCDHEIGHT      64
#define SSD1306_SETCONTRAST   0x81
#define SSD1306_DISPLAYALLON_RESUME 0xA4
#define SSD1306_DISPLAYALLON 0xA5
#define SSD1306_NORMALDISPLAY 0xA6
#define SSD1306_INVERTDISPLAY 0xA7
#define SSD1306_DISPLAYOFF 0xAE
#define SSD1306_DISPLAYON 0xAF
#define SSD1306_SETDISPLAYOFFSET 0xD3
#define SSD1306_SETCOMPINS 0xDA
#define SSD1306_SETVCOMDETECT 0xDB
#define SSD1306_SETDISPLAYCLOCKDIV 0xD5
#define SSD1306_SETPRECHARGE 0xD9
#define SSD1306_SETMULTIPLEX 0xA8
#define SSD1306_SETLOWCOLUMN 0x00
#define SSD1306_SETHIGHCOLUMN 0x10
#define SSD1306_SETSTARTLINE 0x40
#define SSD1306_MEMORYMODE 0x20
#define SSD1306_COLUMNADDR 0x21
#define SSD1306_PAGEADDR   0x22
#define SSD1306_COMSCANINC 0xC0
#define SSD1306_COMSCANDEC 0xC8
#define SSD1306_SEGREMAP 0xA0
#define SSD1306_CHARGEPUMP 0x8D
#define SSD1306_EXTERNALVCC 0x1
#define SSD1306_SWITCHCAPVCC 0x2

typedef uint8_t bitmap_t[8][128];
uint8_t _i2c_address;
void  InitializeDisplay();
void sendStrXY( char *string, int X, int Y);
void display_sendString( char *string);
void setXY(unsigned char row,unsigned char col);
void sendCharXY(unsigned char data, int X, int Y);
void SendChar(unsigned char data);
void display_turnOn(void);
void display_turnOff(void);
void dispaly_clear(void);
void printBigTime(char *string);
void reset_display(void);
void printBigNumber(int num, int X, int Y);
void bmp(bitmap_t b);
void setPageAddress();
void setColAddress();

void printTallyNumber(int camera, int X, int Y);
void display_sendChar(unsigned char data);

#endif /* SSD1306_H_ */