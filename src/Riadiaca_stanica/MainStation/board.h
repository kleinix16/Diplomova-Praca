/*
 * board.h
 *
 * Created: 6/3/2018 1:47:07 PM
 *  Author: klein
 */ 


#ifndef BOARD_H_
#define BOARD_H_

#ifndef F_CPU
/*! \brief Define default CPU frequency, if this is not already defined. */
#define F_CPU 2000000UL
#endif

#include <avr/io.h>

#define sbi(x,y) x |= _BV(y)	 //set bit     (1<<y)
#define cbi(x,y) x &= ~(_BV(y)) //clear bit - using bitwise AND operator
#define tbi(x,y) x ^= _BV(y)    //change bit - using bitwise XOR operator


//***********SYSTEM STATUS****
#define NORMAL          0x00
#define REFRESH         0x01	//Automaticka sprava, pre kontrolu funkcnosti komunikacie
#define CHANGED         0x02	//Sprava generovana pri zmene kamier
#define MESSAGE_BASIC	0x04	//Sprava od rezisera - predpripravena
#define MESSAGE_ADVANCE	0x08	//Sprava od rezisera - napisana
#define RESPONSE		0x10	//Odpoved kameramanov

//***********TIMES****
#define SYSTEM_REFRESH 10
#define BEACON_REFRESH 0x2710			//5000ms  - pouziva sa delicka 1024
#define OUTPUT_PIN_REFRESH 0x0190		//200ms   - pouziva sa delicka 1024

#define RX_NUM_BYTES  128

#define R_LED  PORTD0
#define B_LED  PORTD1
#define G_LED  PORTD2

#define USART_END_CHAR 0xFF

#define USART_BLT USARTC1
#define USART_PORT_BLT PORTC

#define BLT_RX  PORTC7
#define BLT_TX  PORTC6
#define BLT_STATE  PORTC5

#define USB_DP  PORTD7
#define USB_DM  PORTD6

#define USART_PC USARTE0
#define USART_PORT_PC PORTE
#define PC_RX  PORTE2
#define PC_TX  PORTE3

#define USART_LORA USARTC0
#define USART_PORT_LORA PORTC

#define LORA_RX  PORTC3
#define LORA_TX  PORTC2
#define LORA_M0  PORTC0
#define LORA_M1  PORTC1
#define LORA_AUX  PORTC4

#define TALLY_R1  PORTA0
#define TALLY_R2  PORTA1
#define TALLY_R3  PORTA2
#define TALLY_R4  PORTA3
#define TALLY_R5  PORTA4
#define TALLY_R6  PORTA5

#define TALLY_L1  PORTA6
#define TALLY_L2  PORTA7
#define TALLY_L3  PORTB0
#define TALLY_L4  PORTB1
#define TALLY_L5  PORTB2
#define TALLY_L6  PORTB3

#endif /* BOARD_H_ */