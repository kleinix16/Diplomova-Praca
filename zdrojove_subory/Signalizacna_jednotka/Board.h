/*
 *  Board.h
 *  Author: Tomas Klein
 */ 


#ifndef BOARD_H_
#define BOARD_H_
#ifndef F_CPU
#define F_CPU 12000000UL
#endif

//***********CAMERA****
#define CAMERA 3
#define CAMERA_MASK 0x04		//bit representation of camera - 0b00000100

//***********MESSENGE STATUS****
#define REFRESH			 0x01	//automatic generated message
#define CHANGED			 0x02	//change camera state message
#define MESSAGE_BASIC	 0x04	//prepared text message
#define MESSAGE_ADVANCED 0x08	//manual written text message
#define RESPONSE		 0x10	//response from cameraman

/************MESS************/
#define YES    0x0F
#define NO     0xF0

/*********TIMES/TIMER********/
#define WATCHDOG_ERROR_TIME 150
#define FATAL_ERROR_TIME 1000
#define MAIN_LOOP_TIME 100

#define SHOWMESSENGE_ISR_CMP 0xFF
#define SHOWMESSENGE_ISR_CNT 0x1000

#define WATCHDOG_ISR_CMP 0x9C
#define WATCHDOG_ISR_CNT 0x0320

/****RFM comunication****/
#define RFM_PORT PORTD
#define RFM_DDR DDRD

#define RFM_TX PORTD1
#define RFM_RX PORTD0

#define RFM_END_CHAR 0x7E

#define BAUDERATE 9600   
#define UBRR_VALUE ((F_CPU / (BAUDERATE * 8L)) -1)  

#define RFM_M0  PORTD2
#define RFM_M1  PORTD4
#define RFM_AUX PORTD7

/**********I2C***********/
#define I2C_PORT PORTC
#define SDA PORTC4
#define SCK PORTC5

/**********RGB***********/
#define LED_PORT PORTD
#define LED_DDR DDRD

#define B_LED PORTD3
#define G_LED PORTD5
#define R_LED PORTD6

/*********BUTTON***********/
#define BUTTON_PORT PORTB
#define BUTTON_DDR DDRB
#define BUTTON_PIN PINB

#define BT_1 PORTB0
#define BT_2 PORTB1
#define BT_3 PORTB2

/**********MACRO**********/
#define sbi(x,y) x |= _BV(y)	//set bit     (1<<y)
#define cbi(x,y) x &= ~(_BV(y)) //clear bit  - using bitwise AND operator
#define tbi(x,y) x ^= _BV(y)    //change bit - using bitwise XOR operator


#endif /* BOARD_H_ */