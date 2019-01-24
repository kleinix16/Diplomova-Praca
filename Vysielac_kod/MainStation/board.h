/*
 * board.h
 *
 * Created: 6/3/2018 1:47:07 PM
 *  Author: klein
 */ 


#ifndef BOARD_H_
#define BOARD_H_



#include <avr/io.h>

#define sbi(x,y) x |= _BV(y)	 //set bit     (1<<y)
#define cbi(x,y) x &= ~(_BV(y)) //clear bit - using bitwise AND operator
#define tbi(x,y) x ^= _BV(y)    //change bit - using bitwise XOR operator

#define R_LED  PORTD0
#define B_LED  PORTD1
#define G_LED  PORTD2

#define BLT_RX  PORTC7
#define BLT_TX  PORTC6
#define BLT_STATE  PORTC5

#define USB_DP  PORTD7
#define USB_DM  PORTD6

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