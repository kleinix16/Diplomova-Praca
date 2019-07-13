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
#include <math.h>
#include <string.h>

#ifndef F_CPU
#define F_CPU 12000000UL
#endif

#include "I2C.h"
#include <util/delay.h>
#include <avr/pgmspace.h>
#include "dispalyData.h"
#include "ssd1306.h"

uint8_t _i2c_address = 0x78; //display write address

void ssd1306_command(uint8_t c)
{
	uint8_t control = 0x00; // some use 0X00 other examples use 0X80. I tried both
	I2C_Start(_i2c_address);
	//I2C_Write();
	I2C_Write(control); // This is Command
	I2C_Write(c);
	I2C_Stop();
}

void ssd1306_data(uint8_t c)
{
	I2C_Start(_i2c_address);
	I2C_Write(_i2c_address);
	I2C_Write(0X40); // This byte is DATA
	I2C_Write(c);
	I2C_Stop();
}

// Used when doing Horizontal or Vertical Addressing
void setColAddress()
{
	ssd1306_command(SSD1306_COLUMNADDR);   // 0x21 COMMAND
	ssd1306_command(0);					   // Column start address
	ssd1306_command(SSD1306_LCDWIDTH - 1); // Column end address
}

// Used when doing Horizontal or Vertical Addressing
void setPageAddress()
{
	ssd1306_command(SSD1306_PAGEADDR);			  // 0x22 COMMAND
	ssd1306_command(0);							  // Start Page address
	ssd1306_command((SSD1306_LCDHEIGHT / 8) - 1); // End Page address
}

// init according to SSD1306 data sheet and many places on the web
void InitializeDisplay()
{

	PORTD |= (1 << PD4); // set RES high shortly
	DDRD |= (1 << PD4);
	_delay_ms(40);
	PORTD ^= (1 << PD4); // set RES low (active low)
	_delay_ms(40);
	PORTD ^= (1 << PD4); // set RES high
	_delay_ms(40);

	// Init sequence for 128x64 OLED module
	ssd1306_command(SSD1306_DISPLAYOFF); // 0xAE

	ssd1306_command(SSD1306_SETDISPLAYCLOCKDIV); // 0xD5
	ssd1306_command(0x80);						 // the suggested ratio 0x80

	ssd1306_command(SSD1306_SETMULTIPLEX); // 0xA8
	ssd1306_command(0x3F);

	ssd1306_command(SSD1306_SETDISPLAYOFFSET); // 0xD3
	ssd1306_command(0x0);					   // no offset

	ssd1306_command(SSD1306_SETSTARTLINE); // | 0x0);        // line #0

	ssd1306_command(SSD1306_CHARGEPUMP); // 0x8D
	ssd1306_command(0x14);				 // using internal VCC

	ssd1306_command(SSD1306_SEGREMAP | 0x1); // rotate screen 180

	ssd1306_command(SSD1306_COMSCANDEC); // rotate screen 180

	ssd1306_command(SSD1306_SETCOMPINS); // 0xDA
	ssd1306_command(0x12);

	ssd1306_command(SSD1306_SETCONTRAST); // 0x81
	ssd1306_command(0xCF);

	ssd1306_command(SSD1306_SETPRECHARGE); // 0xd9
	ssd1306_command(0xF1);

	ssd1306_command(SSD1306_SETVCOMDETECT); // 0xDB
	ssd1306_command(0x40);

	ssd1306_command(SSD1306_DISPLAYALLON_RESUME); // 0xA4

	ssd1306_command(SSD1306_NORMALDISPLAY); // 0xA6

	ssd1306_command(SSD1306_DISPLAYON); //switch on OLED
}


void bmp(bitmap_t b)
{
	int i, j;

	ssd1306_command(0x00);
	ssd1306_command(0x10);

	ssd1306_command(0xB0);

	for (j = 0; j < 8; j++)
	{
		for (i = 0; i < 128; i++)
			ssd1306_command(b[j][i]);
		ssd1306_command(0);
		ssd1306_command(0);
		ssd1306_command(0);
		ssd1306_command(0);
	}
}
void reset_display(void)
{
	display_turnOff();
	dispaly_clear();

	display_turnOn();
}

//==========================================================//
// Turns display on.
void display_turnOn(void)
{
	ssd1306_command(0xaf); //display on
}

//==========================================================//
// Turns display off.
void display_turnOff(void)
{
	ssd1306_command(0xae); //display off
}

//==========================================================//
// Clears the display by sending 0 to all the screen map.
void dispaly_clear(void)
{
	unsigned char i, k;
	for (k = 0; k < 8; k++)
	{
		setXY(k, 0);
		{
			for (i = 0; i < 129; i++) //clear all COL
			{
				SendChar(0); //clear all COL
							 //delay(10);
			}
		}
	}
}

//==========================================================//
// Prints a display big number (96 bytes) in coordinates X Y,
// being multiples of 8. This means we have 16 COLS (0-15)
// and 8 ROWS (0-7).
void printReadyNumber(uint8_t camera, int X, int Y)
{
	
	uint8_t cam_print = camera;
	if(camera > 6)
	{
		cam_print = 7;     //Char - unknown camera
	}
	
	for (int j = 0; j < 4; j++)
	{
		setXY(X, Y);
		for (int i = 0; i < widthReadyFont; i++)
		{
			SendChar(pgm_read_byte(readyNumbers[cam_print] + (i + j * widthReadyFont)));
		}
		X++;
	}
}

//==========================================================//
// Actually this sends a byte, not a char to draw in the display.
// Display's chars uses 8 byte font the small ones and 96 bytes
// for the big number font.
void SendChar(unsigned char data)
{
	I2C_Start(_i2c_address); // begin transmitting
	I2C_Write(0x40);		 //data mode
	I2C_Write(data);
	I2C_Stop(); // stop transmitting
}

//==========================================================//
// Prints a display char (not just a byte) in coordinates X Y,
// being multiples of 8. This means we have 16 COLS (0-15)
// and 8 ROWS (0-7).
void sendCharXY(unsigned char data, int X, int Y)
{
	setXY(X, Y);
	I2C_Start(_i2c_address); // begin transmitting
	I2C_Write(0x40);		 //data mode

	for (int i = 0; i < 8; i++)
		I2C_Write(pgm_read_byte(smallFont[data - 0x20] + i));

	I2C_Stop(); // stop transmitting
}

//==========================================================//c
// Set the cursor position in a 16 COL * 8 ROW map.
void setXY(unsigned char row, unsigned char col)
{
	ssd1306_command(0xb0 + row);					 //set page address
	ssd1306_command(0x02 + (8 * col & 0x0f));		 //set low col address
	ssd1306_command(0x10 + ((8 * col >> 4) & 0x0f)); //set high col address
}

//==========================================================//
// Prints a string regardless the cursor position.
void display_sendString(char *string)
{
	unsigned char i = 0;
	while (*string)
	{
		for (i = 0; i < 8; i++)
		{
			SendChar(pgm_read_byte(smallFont[*string - 0x20] + i));
		}
		string++;
	}
}

//==========================================================//
// Prints a string in coordinates X Y, being multiples of 8.
// This means we have 16 COLS (0-15) and 8 ROWS (0-7).
void sendStrXY(char *string, int X, int Y)
{
	setXY(X, Y);
	unsigned char i = 0;
	while (*string)
	{
		for (i = 0; i < 8; i++)
		{
			SendChar(pgm_read_byte(smallFont[*string - 0x20] + i));
		}
		string++;
	}
}


//==========================================================//
// Prints a display TALLY big number in coordinates X Y
void printLiveNumber(uint8_t camera, int X, int Y)
{
	uint8_t cam_print = camera;
	if(camera > 6)
	{
		cam_print = 7;
	}
	for (int j = 0; j < 8; j++)
	{
		setXY(X, Y);
		for (int i = 0; i < widthLiveFont; i++)
		{
			SendChar(pgm_read_byte(liveNumbers[cam_print] + (i + j * widthLiveFont)));
		}
		X++;
	}
}

//==========================================================//
// Prints a display TALLY big number in coordinates X Y
void printBootLogo()
{

	uint8_t X = 0;
	for (int j = 0; j < 8; j++)
	{
		setXY(X, 0);
		for (int i = 0; i < 128; i++)
		{
			SendChar(pgm_read_byte(bootLogo[j]+(i)));
		}
		X++;
	}

}


//==========================================================//
// Prints a char in coordinates X Y, being multiples of 8.
// This means we have 16 COLS (0-15) and 8 ROWS (0-7).
void display_sendChar(unsigned char data)
{
	I2C_Start(_i2c_address); // begin transmitting
	I2C_Write(0x40);		 //data mode

	for (int i = 0; i < 8; i++)
		I2C_Write(pgm_read_byte(smallFont[data - 0x20] + i));

	I2C_Stop(); // stop transmitting
}

