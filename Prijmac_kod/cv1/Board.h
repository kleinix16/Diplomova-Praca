/*
 * Board.h
 *
 * 
 *  Author: Tomas Klein
 */ 


#ifndef BOARD_H_
#define BOARD_H_
#ifndef F_CPU
#define F_CPU 12000000UL
#endif

//***********CAMERA****
#define CAMERA 2
#define CAMERA_MASK 0x02

//***********MESS STATUS****
#define REFRESH			0x01	//Automaticka sprava, pre kontrolu funkcnosti komunikacie
#define CHANGED			0x02	//Sprava generovana pri zmene kamier
#define MESSAGE_BASIC	65	//Sprava od rezisera - predpripravena
#define MESSAGE_ADVANCE 70	//Sprava od rezisera - napisana
#define RESPONSE		0x08	//Odpoved kameramanov
#define ERROR			0xFF	//Chybna sprava


//***********TIMES****
#define WATCHDOG_ERROR 150
#define FATAL_ERROR 1000

#define CHECK_MAIN_STATUS 100
#define FATAL_ERROR 1000

#define WATCHDOG_ISR_CMP 0x9C
#define WATCHDOG_ISR_CNT 0x0320


//***********RFM / USART****
#define USART_PORT PORTD
#define RFM_PORT PORTD

#define USART_TX PORTD1
#define USART_RX PORTD0

#define USART_BUFFER 10
#define USART_END_CHAR 0x61

#define BAUDERATE 9600   
#define UBRR_VALUE ((F_CPU / (BAUDERATE * 8L)) -1)  //   8....51  ,  16.....25  12.....12

#define RFM_M0 PORTC2	
#define RFM_M1 PORTC3

#define RFM_AUX PORTD6

//************I2C*****
#define I2C_PORT PORTC
#define SDA PORTC4
#define SCK PORTC5

//**********RGB***
#define LED_PORT PORTD
#define LED_DDR DDRD

#define R_LED PORTD6
#define G_LED PORTD3
#define B_LED PORTB1
//#define B_LED PORTD3
//#define G_LED PORTD5
//#define R_LED PORTD6


//**********TLACIDLA****
#define BUTTON_PORT PORTB
#define BUTTON_DDR DDRB
#define BUTTON_PIN PINB

#define BT_1 PORTB0
#define BT_2 PORTB1
#define BT_3 PORTB2

//**********MACRO******
#define sbi(x,y) x |= _BV(y)	 //nastav bit     (1<<y)
#define cbi(x,y) x &= ~(_BV(y)) //nuluj bit - using bitwise AND operator
#define tbi(x,y) x ^= _BV(y)    //zmen bit - using bitwise XOR operator


#endif /* BOARD_H_ */