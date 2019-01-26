/* 
 * This file contains all the static arrays to draw things in the display.
 *you can create your own fonts using this tool https://www.mikroe.com/glcd-font-creator
 */

#include <avr/pgmspace.h>
typedef uint8_t bitmap_t[8][128];
typedef char PROGMEM prog_uchar;

// Big numbers font, from 0 to 9. 96 byte each.
const prog_uchar bigNumbers[][96] PROGMEM = {
	{0x00, 0x00, 0x00, 0xC0, 0xE0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0,
	 0xF0, 0xF0, 0xF0, 0xE0, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x01,
	 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00,
	 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	 0x80, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x07, 0x0F, 0x0F, 0x0F,
	 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x07, 0x03, 0x00, 0x00, 0x00},

	{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0xF0,
	 0xF0, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF,
	 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x07, 0x07, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},

	{0x00, 0x00, 0x00, 0xE0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0,
	 0xF0, 0xF0, 0xF0, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x81, 0xC1, 0xC0, 0xC0, 0xC0,
	 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xE1, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00,
	 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x87, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03,
	 0x03, 0x03, 0x83, 0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x0F, 0x0F, 0x0F, 0x0F,
	 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x07, 0x00, 0x00, 0x00, 0x00},

	{0x00, 0x00, 0x00, 0xE0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0,
	 0xF0, 0xF0, 0xF0, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x81, 0xC1, 0xC0, 0xC0, 0xC0,
	 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xE1, 0xFF, 0xFF, 0xFF, 0x7F, 0x00, 0x00, 0x00, 0x00,
	 0x00, 0x00, 0x00, 0x81, 0x83, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x87,
	 0xFF, 0xFF, 0xFF, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x0F, 0x0F, 0x0F, 0x0F,
	 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x07, 0x00, 0x00, 0x00, 0x00},

	{0x00, 0x00, 0x00, 0xE0, 0xF0, 0xF0, 0xF0, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
	 0xF0, 0xF0, 0xF0, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xE0,
	 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xE0, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00,
	 0x00, 0x00, 0x00, 0x01, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x07,
	 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0F, 0x0F, 0x0F, 0x07, 0x00, 0x00, 0x00, 0x00},

	{0x00, 0x00, 0x00, 0xE0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0,
	 0xF0, 0xF0, 0xF0, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xE1,
	 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC1, 0x81, 0x00, 0x00, 0x00, 0x00,
	 0x00, 0x00, 0x00, 0x81, 0x83, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x87,
	 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x0F, 0x0F, 0x0F, 0x0F,
	 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x07, 0x00, 0x00, 0x00, 0x00},

	{0x00, 0x00, 0x00, 0xE0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0,
	 0xF0, 0xF0, 0xF0, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xE1,
	 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC1, 0x81, 0x00, 0x00, 0x00, 0x00,
	 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x87, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x87,
	 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x0F, 0x0F, 0x0F, 0x0F,
	 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x07, 0x00, 0x00, 0x00, 0x00},

	{0x00, 0x00, 0x00, 0xE0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0,
	 0xF0, 0xF0, 0xF0, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00,
	 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00,
	 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0F, 0x0F, 0x0F, 0x07, 0x00, 0x00, 0x00, 0x00},

	{0x00, 0x00, 0x00, 0xE0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0,
	 0xF0, 0xF0, 0xF0, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7F, 0xFF, 0xFF, 0xFF, 0xE1,
	 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xE1, 0xFF, 0xFF, 0xFF, 0x3F, 0x00, 0x00, 0x00, 0x00,
	 0x00, 0x00, 0x00, 0xFE, 0xFF, 0xFF, 0xFF, 0x87, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x87,
	 0xFF, 0xFF, 0xFF, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x0F, 0x0F, 0x0F, 0x0F,
	 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x07, 0x00, 0x00, 0x00, 0x00},

	{0x00, 0x00, 0x00, 0xE0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0,
	 0xF0, 0xF0, 0xF0, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xE1,
	 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xE1, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00,
	 0x00, 0x00, 0x00, 0x01, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x07,
	 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0F, 0x0F, 0x0F, 0x07, 0x00, 0x00, 0x00, 0x00},

	{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	 0x00, 0x3C, 0x7E, 0x7E, 0x7E, 0x7E, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0xF8, 0xF8, 0xF8, 0xF8, 0xF0, 0x00,
	 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};

// Big numbers minus symbol.
const prog_uchar minus[] PROGMEM = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x0C, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E,
	0x1E, 0x1E, 0x1E, 0x1E, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// Small 8x8 font
const prog_uchar myFont[][8] PROGMEM = {
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
	{0x00, 0x00, 0x5F, 0x00, 0x00, 0x00, 0x00, 0x00},
	{0x00, 0x00, 0x07, 0x00, 0x07, 0x00, 0x00, 0x00},
	{0x00, 0x14, 0x7F, 0x14, 0x7F, 0x14, 0x00, 0x00},
	{0x00, 0x24, 0x2A, 0x7F, 0x2A, 0x12, 0x00, 0x00},
	{0x00, 0x23, 0x13, 0x08, 0x64, 0x62, 0x00, 0x00},
	{0x00, 0x36, 0x49, 0x55, 0x22, 0x50, 0x00, 0x00},
	{0x00, 0x00, 0x05, 0x03, 0x00, 0x00, 0x00, 0x00},
	{0x00, 0x1C, 0x22, 0x41, 0x00, 0x00, 0x00, 0x00},
	{0x00, 0x41, 0x22, 0x1C, 0x00, 0x00, 0x00, 0x00},
	{0x00, 0x08, 0x2A, 0x1C, 0x2A, 0x08, 0x00, 0x00},
	{0x00, 0x08, 0x08, 0x3E, 0x08, 0x08, 0x00, 0x00},
	{0x00, 0xA0, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00},
	{0x00, 0x08, 0x08, 0x08, 0x08, 0x08, 0x00, 0x00},
	{0x00, 0x60, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00},
	{0x00, 0x20, 0x10, 0x08, 0x04, 0x02, 0x00, 0x00},
	{0x00, 0x3E, 0x51, 0x49, 0x45, 0x3E, 0x00, 0x00},
	{0x00, 0x00, 0x42, 0x7F, 0x40, 0x00, 0x00, 0x00},
	{0x00, 0x62, 0x51, 0x49, 0x49, 0x46, 0x00, 0x00},
	{0x00, 0x22, 0x41, 0x49, 0x49, 0x36, 0x00, 0x00},
	{0x00, 0x18, 0x14, 0x12, 0x7F, 0x10, 0x00, 0x00},
	{0x00, 0x27, 0x45, 0x45, 0x45, 0x39, 0x00, 0x00},
	{0x00, 0x3C, 0x4A, 0x49, 0x49, 0x30, 0x00, 0x00},
	{0x00, 0x01, 0x71, 0x09, 0x05, 0x03, 0x00, 0x00},
	{0x00, 0x36, 0x49, 0x49, 0x49, 0x36, 0x00, 0x00},
	{0x00, 0x06, 0x49, 0x49, 0x29, 0x1E, 0x00, 0x00},
	{0x00, 0x00, 0x36, 0x36, 0x00, 0x00, 0x00, 0x00},
	{0x00, 0x00, 0xAC, 0x6C, 0x00, 0x00, 0x00, 0x00},
	{0x00, 0x08, 0x14, 0x22, 0x41, 0x00, 0x00, 0x00},
	{0x00, 0x14, 0x14, 0x14, 0x14, 0x14, 0x00, 0x00},
	{0x00, 0x41, 0x22, 0x14, 0x08, 0x00, 0x00, 0x00},
	{0x00, 0x02, 0x01, 0x51, 0x09, 0x06, 0x00, 0x00},
	{0x00, 0x32, 0x49, 0x79, 0x41, 0x3E, 0x00, 0x00},
	{0x00, 0x7E, 0x09, 0x09, 0x09, 0x7E, 0x00, 0x00},
	{0x00, 0x7F, 0x49, 0x49, 0x49, 0x36, 0x00, 0x00},
	{0x00, 0x3E, 0x41, 0x41, 0x41, 0x22, 0x00, 0x00},
	{0x00, 0x7F, 0x41, 0x41, 0x22, 0x1C, 0x00, 0x00},
	{0x00, 0x7F, 0x49, 0x49, 0x49, 0x41, 0x00, 0x00},
	{0x00, 0x7F, 0x09, 0x09, 0x09, 0x01, 0x00, 0x00},
	{0x00, 0x3E, 0x41, 0x41, 0x51, 0x72, 0x00, 0x00},
	{0x00, 0x7F, 0x08, 0x08, 0x08, 0x7F, 0x00, 0x00},
	{0x00, 0x41, 0x7F, 0x41, 0x00, 0x00, 0x00, 0x00},
	{0x00, 0x20, 0x40, 0x41, 0x3F, 0x01, 0x00, 0x00},
	{0x00, 0x7F, 0x08, 0x14, 0x22, 0x41, 0x00, 0x00},
	{0x00, 0x7F, 0x40, 0x40, 0x40, 0x40, 0x00, 0x00},
	{0x00, 0x7F, 0x02, 0x0C, 0x02, 0x7F, 0x00, 0x00},
	{0x00, 0x7F, 0x04, 0x08, 0x10, 0x7F, 0x00, 0x00},
	{0x00, 0x3E, 0x41, 0x41, 0x41, 0x3E, 0x00, 0x00},
	{0x00, 0x7F, 0x09, 0x09, 0x09, 0x06, 0x00, 0x00},
	{0x00, 0x3E, 0x41, 0x51, 0x21, 0x5E, 0x00, 0x00},
	{0x00, 0x7F, 0x09, 0x19, 0x29, 0x46, 0x00, 0x00},
	{0x00, 0x26, 0x49, 0x49, 0x49, 0x32, 0x00, 0x00},
	{0x00, 0x01, 0x01, 0x7F, 0x01, 0x01, 0x00, 0x00},
	{0x00, 0x3F, 0x40, 0x40, 0x40, 0x3F, 0x00, 0x00},
	{0x00, 0x1F, 0x20, 0x40, 0x20, 0x1F, 0x00, 0x00},
	{0x00, 0x3F, 0x40, 0x38, 0x40, 0x3F, 0x00, 0x00},
	{0x00, 0x63, 0x14, 0x08, 0x14, 0x63, 0x00, 0x00},
	{0x00, 0x03, 0x04, 0x78, 0x04, 0x03, 0x00, 0x00},
	{0x00, 0x61, 0x51, 0x49, 0x45, 0x43, 0x00, 0x00},
	{0x00, 0x7F, 0x41, 0x41, 0x00, 0x00, 0x00, 0x00},
	{0x00, 0x02, 0x04, 0x08, 0x10, 0x20, 0x00, 0x00},
	{0x00, 0x41, 0x41, 0x7F, 0x00, 0x00, 0x00, 0x00},
	{0x00, 0x04, 0x02, 0x01, 0x02, 0x04, 0x00, 0x00},
	{0x00, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00},
	{0x00, 0x01, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00},
	{0x00, 0x20, 0x54, 0x54, 0x54, 0x78, 0x00, 0x00},
	{0x00, 0x7F, 0x48, 0x44, 0x44, 0x38, 0x00, 0x00},
	{0x00, 0x38, 0x44, 0x44, 0x28, 0x00, 0x00, 0x00},
	{0x00, 0x38, 0x44, 0x44, 0x48, 0x7F, 0x00, 0x00},
	{0x00, 0x38, 0x54, 0x54, 0x54, 0x18, 0x00, 0x00},
	{0x00, 0x08, 0x7E, 0x09, 0x02, 0x00, 0x00, 0x00},
	{0x00, 0x18, 0xA4, 0xA4, 0xA4, 0x7C, 0x00, 0x00},
	{0x00, 0x7F, 0x08, 0x04, 0x04, 0x78, 0x00, 0x00},
	{0x00, 0x00, 0x7D, 0x00, 0x00, 0x00, 0x00, 0x00},
	{0x00, 0x80, 0x84, 0x7D, 0x00, 0x00, 0x00, 0x00},
	{0x00, 0x7F, 0x10, 0x28, 0x44, 0x00, 0x00, 0x00},
	{0x00, 0x41, 0x7F, 0x40, 0x00, 0x00, 0x00, 0x00},
	{0x00, 0x7C, 0x04, 0x18, 0x04, 0x78, 0x00, 0x00},
	{0x00, 0x7C, 0x08, 0x04, 0x7C, 0x00, 0x00, 0x00},
	{0x00, 0x38, 0x44, 0x44, 0x38, 0x00, 0x00, 0x00},
	{0x00, 0xFC, 0x24, 0x24, 0x18, 0x00, 0x00, 0x00},
	{0x00, 0x18, 0x24, 0x24, 0xFC, 0x00, 0x00, 0x00},
	{0x00, 0x00, 0x7C, 0x08, 0x04, 0x00, 0x00, 0x00},
	{0x00, 0x48, 0x54, 0x54, 0x24, 0x00, 0x00, 0x00},
	{0x00, 0x04, 0x7F, 0x44, 0x00, 0x00, 0x00, 0x00},
	{0x00, 0x3C, 0x40, 0x40, 0x7C, 0x00, 0x00, 0x00},
	{0x00, 0x1C, 0x20, 0x40, 0x20, 0x1C, 0x00, 0x00},
	{0x00, 0x3C, 0x40, 0x30, 0x40, 0x3C, 0x00, 0x00},
	{0x00, 0x44, 0x28, 0x10, 0x28, 0x44, 0x00, 0x00},
	{0x00, 0x1C, 0xA0, 0xA0, 0x7C, 0x00, 0x00, 0x00},
	{0x00, 0x44, 0x64, 0x54, 0x4C, 0x44, 0x00, 0x00},
	{0x00, 0x08, 0x36, 0x41, 0x00, 0x00, 0x00, 0x00},
	{0x00, 0x00, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x00},
	{0x00, 0x41, 0x36, 0x08, 0x00, 0x00, 0x00, 0x00},
	{0x00, 0x02, 0x01, 0x01, 0x02, 0x01, 0x00, 0x00},
	{0x00, 0x02, 0x05, 0x05, 0x02, 0x00, 0x00, 0x00}
};

const uint8_t boot_logo[8][128] PROGMEM = {
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xC0, 0xE0, 0x70, 0x38, 0x1C, 0x0C, 0x0C, 0x1C, 0x38, 0x70, 0x20, 0x00, 0x00, 0x80, 0xC0, 0xE0, 0x70, 0x38, 0x1C, 0x0C, 0x0C, 0x1C, 0x38, 0x70, 0xE0, 0xC0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x32, 0x49, 0x49, 0x49, 0x26, 0x00, 0x7F, 0x20, 0x10, 0x20, 0x7F, 0x00, 0x3F, 0x44, 0x44, 0x44, 0x3F, 0x00, 0x7F, 0x01, 0x01, 0x01, 0x01, 0x00, 0x7F, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1E, 0x29, 0x49, 0x49, 0x06, 0x00, 0x1B, 0x04, 0x04, 0x1B, 0x00, 0x00, 0x36, 0x49, 0x49, 0x49, 0x36, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x40, 0x7F, 0x40, 0x40, 0x00, 0x7F, 0x49, 0x49, 0x49, 0x41, 0x80, 0xE3, 0x94, 0x88, 0x94, 0x63, 0x00, 0x40, 0x40, 0x7F, 0x40, 0x40},
	{0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x98, 0x9C, 0x9E, 0x9F, 0x9F, 0x9F, 0x9F, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0xF8, 0xFC, 0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x9F, 0x9F, 0x9F, 0x9F, 0x9E, 0x9C, 0x98, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 0x80, 0x00, 0x00, 0x80, 0x00, 0x00, 0x80, 0x00, 0x00, 0x80, 0x00, 0x00, 0x80, 0x00, 0x00, 0x80, 0x00, 0x00, 0x80, 0x00, 0x00, 0x80, 0x00, 0x00, 0x80, 0x80, 0x80, 0xC0, 0xE0, 0x60, 0xE0, 0x60, 0xE0, 0xC0, 0x80, 0x00, 0x80, 0x00, 0x00, 0x80, 0x00, 0x00, 0x80, 0x00, 0x00, 0x80, 0x00, 0x00, 0x80, 0x00, 0x00, 0x80, 0x00, 0x00, 0x80, 0x00, 0x00, 0x80, 0x00, 0x00, 0x80, 0x00, 0xC0, 0xE0, 0x78, 0x1C, 0x8F, 0x03, 0x01, 0x81, 0x01, 0x03, 0x8F, 0x1E, 0xF8, 0xE0, 0x80, 0x00, 0x80},
	{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x99, 0x99, 0x99, 0x99, 0x99, 0x99, 0x99, 0x99, 0x99, 0x99, 0x99, 0x99, 0x99, 0x99, 0x99, 0x99, 0x99, 0x99, 0x99, 0x00, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0xC0, 0xF0, 0xB8, 0x9E, 0x8F, 0x83, 0x81, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x81, 0x83, 0x87, 0x9E, 0xB8, 0xF0, 0xC0, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0xE0, 0xF8, 0xBE, 0x8F, 0x83, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x83, 0x8F, 0xFE, 0xF8},
	{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x99, 0x99, 0x99, 0x99, 0x99, 0x99, 0x99, 0x99, 0x99, 0x99, 0x99, 0x99, 0x99, 0x99, 0x99, 0x99, 0x99, 0x99, 0x99, 0x00, 0x03, 0x07, 0x0D, 0x1D, 0x39, 0x71, 0x61, 0xC1, 0xC1, 0x81, 0x81, 0x81, 0xC1, 0xC1, 0xE1, 0x71, 0x39, 0x1D, 0x0F, 0x07, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x03, 0x0F, 0x3D, 0xF9, 0xE1, 0x81, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0xC1, 0xF1, 0x7D, 0x1F, 0x07, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01},
	{0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x19, 0x39, 0x79, 0xF9, 0xF9, 0xF9, 0xF9, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x1F, 0x3F, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0xF9, 0xF9, 0xF9, 0xF9, 0x79, 0x39, 0x19, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x01, 0x03, 0x07, 0x0F, 0x1C, 0x38, 0x71, 0x60, 0x60, 0x71, 0x38, 0x1C, 0x0F, 0x07, 0x01, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x01},
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x07, 0x0E, 0x1C, 0x38, 0x30, 0x30, 0x38, 0x1C, 0x0E, 0x04, 0x00, 0x00, 0x01, 0x03, 0x07, 0x0E, 0x1C, 0x38, 0x30, 0x30, 0x38, 0x1C, 0x0E, 0x07, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3C, 0xFE, 0xC3, 0x01, 0x01, 0x03, 0x86, 0x84, 0x03, 0x7F, 0xFC, 0x90, 0x10, 0x10, 0x11, 0x07, 0x1E, 0x7C, 0xE4, 0x84, 0xFC, 0x7F, 0x03, 0x00, 0x07, 0x7F, 0xF9, 0x81, 0x01, 0x01, 0x00, 0x00, 0x00, 0x60, 0xC7, 0xFF, 0xF8, 0x80, 0x00, 0x03, 0x87, 0x8D, 0x19, 0xF1, 0xE0, 0x00, 0x6E, 0xFF, 0x11, 0x11, 0xFE, 0xEC, 0x00, 0x3E, 0xFF, 0xD1, 0x23, 0xBE, 0x9C, 0x0C, 0x1C, 0x34, 0x67, 0xFF, 0xFC, 0x84, 0x00, 0x3C, 0xFE, 0xC3, 0x01, 0x01, 0x03, 0x86, 0x84, 0x00, 0x00},
	{0xC0, 0xF0, 0x38, 0x1C, 0x0C, 0x0C, 0x0C, 0x1C, 0x38, 0x30, 0x00, 0xFC, 0xFC, 0x80, 0x80, 0xC0, 0xE0, 0xF0, 0xB8, 0x1C, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x38, 0x1C, 0x0C, 0x0C, 0x0C, 0x0C, 0x9C, 0xF8, 0xF0, 0x00, 0x00, 0x00, 0x00, 0xFC, 0xFC, 0x00, 0x00, 0x00, 0x1C, 0xFC, 0xE1, 0x61, 0x61, 0x61, 0x60, 0xE0, 0xFC, 0x1D, 0x01, 0x01, 0xFD, 0xFD, 0x0D, 0x0C, 0x0C, 0x0D, 0x0D, 0x0D, 0x0C, 0x0C, 0x00, 0x00, 0xFC, 0xFD, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xF1, 0x39, 0x1C, 0x0C, 0x0C, 0x1D, 0x39, 0xF1, 0xC0, 0x00, 0x00, 0xFC, 0xFD, 0x01, 0x01, 0x00, 0xC0, 0xE0, 0x38, 0xFD, 0xFD, 0x01, 0x00, 0x00, 0x00, 0xFC, 0xFC, 0x00, 0x01, 0x01, 0x00, 0x0C, 0x3C, 0x7C, 0xCD, 0x8D, 0x0D, 0x0D, 0x0C, 0x0C, 0x0C},
	{0x0F, 0x3F, 0x70, 0xE0, 0xC0, 0xC0, 0xC0, 0xE0, 0x70, 0x30, 0x00, 0xFF, 0xFF, 0xC1, 0xC1, 0xC1, 0xC1, 0xC1, 0xE3, 0x7F, 0x3E, 0x00, 0xC0, 0xF0, 0x3C, 0x0F, 0x03, 0x03, 0x0F, 0x3C, 0xF0, 0xC0, 0x00, 0x3C, 0x7E, 0xE7, 0xC3, 0xC3, 0xC3, 0xC3, 0xE3, 0x71, 0x30, 0x00, 0xC0, 0xC0, 0xC0, 0xFF, 0xFF, 0xC0, 0xC0, 0xC0, 0x00, 0x00, 0x07, 0x3F, 0xF8, 0xF8, 0x3F, 0x07, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xC3, 0xC3, 0xC3, 0xC3, 0xC3, 0xC3, 0xC0, 0xC0, 0x0F, 0x3F, 0x70, 0xE0, 0xC0, 0xC0, 0xE0, 0x70, 0x3F, 0x0F, 0x00, 0x00, 0xFF, 0xFF, 0x70, 0x1C, 0x0F, 0x03, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0xC0, 0xC0, 0xC0, 0xFF, 0xFF, 0xC0, 0xC0, 0xC0, 0x00, 0xC0, 0xC0, 0xC0, 0xC0, 0xC3, 0xC7, 0xCC, 0xF8, 0xF0, 0xC0}
};

// TALLY font Roboto_Mono 42 x 64
//http://www.mikroe.com
const int wightTallyFont = 42;

const prog_uchar tallyNumbers[][336] PROGMEM = {
	{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x80, 0xc0, 0xe0, 0xf0, 0xf0, 0xf8, 0xf8, 0xf8, 0xfc, 0xfc, 0xfc, 0xfc, 0xfc, 0xfc, 0xfc, 0xfc, 0xfc, 0xfc, 0xf8, 0xf8, 0xf8, 0xf0, 0xf0, 0xe0, 0xc0, 0x80, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xe0, 0xf8, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x7f, 0x1f, 0xf, 0xf, 0x7, 0x7, 0x7, 0x7, 0x7, 0x7, 0xf, 0xf, 0x1f, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xf8, 0xe0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x80, 0x8f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x80, 0x0, 0x0, 0x0, 0x0, 0x0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x0, 0x0, 0x0, 0x0, 0x80, 0xc0, 0xe0, 0xe0, 0xf0, 0xf8, 0xfc, 0xfc, 0xfe, 0xff, 0x7f, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x0, 0x0, 0x0, 0x0, 0x0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0xfe, 0xff, 0x7f, 0x3f, 0x1f, 0x1f, 0xf, 0x7, 0x7, 0x3, 0x1, 0x0, 0x0, 0x0, 0x0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x0, 0x0, 0x0, 0x0, 0x0, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xf8, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x7f, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x3, 0xf, 0x3f, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0xf8, 0xf8, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf8, 0xf8, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x7f, 0x3f, 0xf, 0x3, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x1, 0x3, 0x7, 0x7, 0xf, 0xf, 0xf, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0xf, 0xf, 0xf, 0x7, 0x7, 0x3, 0x1, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0}, // Code for char 0
	{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x80, 0x80, 0xc0, 0xc0, 0xc0, 0xe0, 0xe0, 0xe0, 0xf0, 0xf0, 0xf0, 0xf8, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xf0, 0xf8, 0xf8, 0xf8, 0xfc, 0xfc, 0xfc, 0xfe, 0xfe, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x3f, 0x1f, 0x1f, 0xf, 0xf, 0x7, 0x7, 0x7, 0x3, 0x3, 0x3, 0x1, 0x1, 0x1, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0},																									   // Code for char 1
	{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x80, 0xc0, 0xc0, 0xe0, 0xf0, 0xf0, 0xf0, 0xf8, 0xf8, 0xf8, 0xfc, 0xfc, 0xfc, 0xfc, 0xfc, 0xfc, 0xfc, 0xfc, 0xfc, 0xfc, 0xf8, 0xf8, 0xf8, 0xf0, 0xf0, 0xe0, 0xe0, 0xc0, 0x80, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x80, 0xe0, 0xf8, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x7f, 0x3f, 0x1f, 0xf, 0x7, 0x7, 0x7, 0x7, 0x7, 0x7, 0x7, 0x7, 0xf, 0xf, 0x3f, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0xf0, 0x80, 0x0, 0x0, 0x0, 0x0, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xc0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x3f, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x80, 0xc0, 0xe0, 0xf8, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x7f, 0x1f, 0x7, 0x1, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x80, 0xc0, 0xe0, 0xf0, 0xfc, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x7f, 0x3f, 0x1f, 0x7, 0x3, 0x1, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xc0, 0xe0, 0xf0, 0xf8, 0xfc, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x7f, 0x3f, 0x1f, 0xf, 0x7, 0x3, 0x1, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xf0, 0xf8, 0xfc, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfb, 0xf9, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0x0, 0x0, 0x0, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0x0},											   //Code for char 2
	{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x80, 0xc0, 0xe0, 0xe0, 0xf0, 0xf0, 0xf8, 0xf8, 0xf8, 0xfc, 0xfc, 0xfc, 0xfc, 0xfc, 0xfc, 0xfc, 0xfc, 0xfc, 0xfc, 0xf8, 0xf8, 0xf8, 0xf8, 0xf0, 0xf0, 0xe0, 0xe0, 0xc0, 0x80, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xf0, 0xfc, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x3f, 0x1f, 0xf, 0x7, 0x7, 0x7, 0x7, 0x7, 0x7, 0x7, 0x7, 0xf, 0xf, 0x1f, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xf8, 0xc0, 0x0, 0x0, 0x0, 0x0, 0x3, 0x3, 0x3, 0x3, 0x3, 0x3, 0x3, 0x3, 0x3, 0x3, 0x3, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xc0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x3f, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xfc, 0xfc, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0x7f, 0x3f, 0x1f, 0x1f, 0xf, 0x3, 0x1, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0x1f, 0x1f, 0x3f, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xfe, 0xfc, 0xf8, 0xf0, 0xc0, 0x0, 0x0, 0x0, 0x0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x81, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x0, 0x0, 0x0, 0x3, 0xf, 0x3f, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0xf8, 0xf8, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf8, 0xf8, 0xfc, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x7f, 0x3f, 0x1f, 0x7, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x1, 0x3, 0x3, 0x7, 0x7, 0xf, 0xf, 0xf, 0xf, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0xf, 0xf, 0xf, 0xf, 0x7, 0x7, 0x3, 0x3, 0x1, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0},						   //Code for char 3
	{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x80, 0xc0, 0xf0, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x80, 0xc0, 0xf0, 0xf8, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x80, 0xc0, 0xf0, 0xfc, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0x7f, 0x1f, 0x7, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x80, 0xc0, 0xf0, 0xfc, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0x3f, 0x1f, 0x7, 0x3, 0x0, 0x0, 0x0, 0x0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x80, 0xc0, 0xf0, 0xfc, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xbf, 0x9f, 0x87, 0x81, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0xe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0},						   // Code for char 4
	{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xf8, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x7, 0x0, 0x0, 0x0, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xe0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xfe, 0xfc, 0xfc, 0xf8, 0xf0, 0xc0, 0x80, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x1, 0x1, 0x1, 0x3, 0x3, 0x3, 0x3, 0x7, 0x7, 0x3, 0x1, 0x1, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x1, 0x3, 0x3, 0xf, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0xc0, 0x0, 0x0, 0x0, 0x0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xc0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xc0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f, 0x0, 0x0, 0x0, 0x0, 0x3, 0xf, 0x1f, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xfc, 0xf8, 0xf8, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf8, 0xf8, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x7f, 0x3f, 0xf, 0x3, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x1, 0x1, 0x3, 0x7, 0x7, 0x7, 0xf, 0xf, 0xf, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0xf, 0xf, 0xf, 0x7, 0x7, 0x3, 0x3, 0x1, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0},						   // Code for char 5
	{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x80, 0x80, 0xc0, 0xe0, 0xe0, 0xf0, 0xf0, 0xf0, 0xf8, 0xf8, 0xf8, 0xf8, 0xfc, 0xfc, 0xfc, 0xfc, 0xfc, 0xfc, 0xfc, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x80, 0xc0, 0xf0, 0xf8, 0xfc, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x7f, 0x7f, 0x3f, 0x1f, 0x1f, 0xf, 0xf, 0xf, 0x7, 0x7, 0x7, 0x7, 0x7, 0x7, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x80, 0xf0, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x3f, 0x7, 0x3, 0x0, 0x0, 0x80, 0x80, 0x80, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0x80, 0x80, 0x80, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xf0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf9, 0xfc, 0xfe, 0xfe, 0xff, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xfc, 0xfc, 0xf0, 0xe0, 0x80, 0x0, 0x0, 0x0, 0x0, 0x0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf, 0x3, 0x1, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x1, 0x3, 0x7, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xe0, 0x0, 0x0, 0x0, 0x7, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xc0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x3f, 0x0, 0x0, 0x0, 0x0, 0x0, 0x3, 0xf, 0x1f, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xfc, 0xf8, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf8, 0xf8, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x7f, 0x1f, 0xf, 0x1, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x1, 0x1, 0x3, 0x7, 0x7, 0xf, 0xf, 0xf, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0xf, 0xf, 0xf, 0x7, 0x7, 0x3, 0x1, 0x1, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0},			   // Code for char 6
	{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0}																																								   // Code for char  -
};