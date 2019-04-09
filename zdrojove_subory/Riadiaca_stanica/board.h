/*
 * board.h
 *
 * Title: Diploma thesis - Communication for cameraman 
 * Created: 6/3/2018 1:47:07 PM
 * Author: Tomas Klein
 */ 


#ifndef BOARD_H_
#define BOARD_H_

#ifndef F_CPU
#define F_CPU 2000000UL
#endif

#include <avr/io.h>

#define sbi(x,y) x |= _BV(y)	 //set bit     (1<<y)
#define cbi(x,y) x &= ~(_BV(y)) //clear bit - using bitwise AND operator
#define tbi(x,y) x ^= _BV(y)    //change bit - using bitwise XOR operator


/***********SYSTEM STATUS****/
#define NORMAL           0x00
#define REFRESH			 0x01	//automatic generated message
#define CHANGED			 0x02	//change camera state message
#define MESSAGE_BASIC	 0x04	//prepared text message
#define MESSAGE_ADVANCED 0x08	//manual written text message
#define RESPONSE		 0x10	//response from cameraman

/***********TIMES*********/
#define SYSTEM_REFRESH 10
#define BEACON_REFRESH 0x2710			//4000ms  - prescale 1024
#define OUTPUT_PIN_REFRESH 0x0190		//200ms   - prescale 1024

/***********LEDS*********/
#define R_LED  PORTD0
#define B_LED  PORTD1
#define G_LED  PORTD2

/***********USART*********/
#define RX_NUM_BYTES  128
#define UART_END_CHAR 0x7E

/*********BLUETOOTH*******/
#define BLT_USART USARTC1
#define BLT_PORT_UART PORTC
#define BLT_RX     PORTC7
#define BLT_TX     PORTC6
#define BLT_STATE  PORTC5

/***********PC COMMUNICATION*********/
#define USART_PC      USARTE0
#define USART_PORT_PC PORTE
#define PC_RX   PORTE2
#define PC_TX   PORTE3

#define USB_DP  PORTD7
#define USB_DM  PORTD6

/***********RF MODULE*********/
#define RFM_UART USARTC0
#define RFM_PORT PORTC

#define RFM_RX   PORTC3
#define RFM_TX   PORTC2
#define RFM_M0   PORTC0
#define RFM_M1   PORTC1
#define RFM_AUX  PORTC4

/***********GPI*********/
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