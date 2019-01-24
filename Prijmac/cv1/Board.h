/*
 * Board.h
 *
 * Created: 16.11.2017 21:19:29
 *  Author: Juraj
 */ 


#ifndef BOARD_H_
#define BOARD_H_
#ifndef F_CPU
#define F_CPU 12000000UL
#endif

//***********CAMERA****
#define CAMERA 2
#define CAMERA_MASK 0x02

//***********RFM / USATR****
#define HTX PORTD1
#define HRX PORTD0

#define USART_BUFFER 10
#define USART_END_CHAR 0x61

#define BAUDERATE 9600   //9600
#define UBRR_VALUE ((F_CPU / (BAUDERATE * 8L)) -1)  //   8....51  ,  16.....25  12.....12

#define M0 PORTC2
#define M1 PORTC3

#define AUX PORTD6

//************I2C*****

//**********RGB***
#define R_LED PORTD6
#define G_LED PORTD3
#define B_LED PORTB1

//**********TLACIDLA****
#define TL_1 PORTD2
#define TL_2 PORTD2
#define TL_3 PORTD2

//**********MACRO******
#define sbi(x,y) x |= _BV(y)	 //nastav bit     (1<<y)
#define cbi(x,y) x &= ~(_BV(y)) //nuluj bit - using bitwise AND operator
#define tbi(x,y) x ^= _BV(y)    //zmen bit - using bitwise XOR operator



#define S_IDLE  0b00000000		// idle
#define S_ADC   0b00010000		//ADC noise reduction
#define S_PWD	0b00100000		//power down
#define S_PWS	0b00110000		//power save
#define S_STD	0b10100000		//stand by
#define S_EST	0b10110000		//extended stand by
#define S_SE	0b01000000		//sleep enable


#endif /* BOARD_H_ */