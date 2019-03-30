/*
* 
* Diploma thesis - Communication for cameraman
*
* Last update: 29/3/2019 
* Author: Tomas Klein
* School: Zilinska univerzita v Ziline
* Description: Program designed for devices fixed to camera. This device will received data from RF communication module and process data as messages. 
*              4 types of messages: - change status of used cameras (LIVE and READY)
*									- refresh message - live connection with control station
*									- predefined text message - received only message ID
*									- manual written text message - received whole message 
*/

#include "Board.h"

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include <math.h>
#include <string.h>
#include "I2C.h"

#include <avr/pgmspace.h>
#include "ssd1306.h"
#include "printMessenge.h"

#include <stdbool.h>
#include <stdio.h>
#include "ringBuffer.h"


circularQueue_t RX_ringBuffer;	// ring buffer for data from RX module		

uint8_t newMessenges = 0;		// number of new unprocessed messages

volatile uint8_t portbHistory = 0xFF;     // last saved value of PORT B

bool systemError = false;	// fatal system error, irretrievable status
bool watchdog = false;		// variable for live connection with other RF module, false is correct status
bool textOnDisplay = false;	// display show text message 

volatile uint8_t readyCameras = 10; // numeric designation for camera with READY status
volatile uint8_t liveCameras = 10;  // numeric designation for camera with LIVE status

uint16_t timerx_T0;    //variable used for timer T0
uint16_t timerx_T2;	   //variable used for timer T2


/************************************************************************/
/* Initialization output pins for RGB LEDs						        */
/* HIGH - LED on, LOW - LED off											*/
/* default - All outputs to HIGH										*/
/************************************************************************/
void init_LED(void)
{
	sbi(LED_DDR, G_LED);  // set pin as output
	sbi(LED_DDR, R_LED);
	sbi(LED_DDR, B_LED);

	sbi(LED_PORT, G_LED); // set output to HIGH
	sbi(LED_PORT, R_LED);
	sbi(LED_PORT, B_LED);
}

/************************************************************************/
/* Initialization input pins for users buttons							*/
/************************************************************************/
void init_button(void)
{
	sbi(BUTTON_PORT, BT_1); // turn On the Pull-up
	sbi(BUTTON_PORT, BT_2);
	sbi(BUTTON_PORT, BT_3);

	cbi(BUTTON_DDR, BT_1);  // set pin as input
	cbi(BUTTON_DDR, BT_2);
	cbi(BUTTON_DDR, BT_3);
	
	sbi(PCICR, PCIE0);		// set PCIE0 to enable PCMSK0 scan
	
	sbi(PCMSK0, PCINT0);	// set PCINT0 to trigger an interrupt on state change 
	sbi(PCMSK0, PCINT1);
	sbi(PCMSK0, PCINT2);	
}

/************************************************************************/
/* Initialization 8-bit timer to control live connection with control station*/
/* use additional 16-bites variable timerx_T0 (possibility count to 10s)*/
/************************************************************************/
void init_T0_WD()
{
	OCR0A = WATCHDOG_ISR_CMP;			// set the value that you want to count to
	TCCR0A |= (1 << WGM01);				// set the Timer Mode to CTC
	TCCR0B |= (1 << CS12) | (1 << CS10);// set prescaler to 1024 and start the timer
	TIMSK0 |= (1 << OCIE0A);			// set interrupt on compare match 
}

/************************************************************************/
/* Initialization 8-bit timer for automatic clean display			    */
/* use additional 16-bites variable timerx_T2 (possibility count to 10s)*/
/************************************************************************/
void init_T2_autoDisplayCleaner()
{
	OCR2A = SHOWMESSENGE_ISR_CMP;		// set the value that you want to count to
	TCCR2A = 0x2;						// set the Timer Mode to CTC
	TCCR2B |= (1 << CS12) | (1 << CS10);// set prescaler to 1024 and start the timer
	TIMSK2 |= (1 << OCIE2A);			// set interrupt on compare match 
}

/************************************************************************/
/* Display communication initialization								    */
/* Init. I2C, connect to display and clear display                      */
/************************************************************************/
void init_display(void)
{
	I2C_Init();							//Initialization I2C interface
	_delay_ms(10);
	
	InitializeDisplay();				
	dispaly_clear();
}

/************************************************************************/
/* Initializaion RF communication modul - UART						    */
/* CDEbyte - E34-2G4H20D									            */
/************************************************************************/
void init_RFM_UART(void)
{
	//RFM AUX
	sbi(RFM_PORT, RFM_AUX); //set RFM AUX as input
	cbi(RFM_DDR, RFM_AUX);
	
	//RFM M0, M1
	sbi(RFM_DDR, RFM_M0);   //set RFM M0 a M1 as output
	sbi(RFM_DDR, RFM_M1);
	
	cbi(RFM_PORT, RFM_M0);  //init to LOW
	cbi(RFM_PORT, RFM_M1);

	//RFM UART RX, TX
	sbi(RFM_DDR, RFM_TX);	//set RFM UART TX as output
	
	sbi(RFM_PORT, RFM_RX);  //set RFM UART RX as input
	cbi(RFM_DDR, RFM_RX);

	UBRR0H = (uint8_t)(UBRR_VALUE >> 8);	 //BAUDERATE
	UBRR0L = (uint8_t)UBRR_VALUE;

	UCSR0B |= (1 << TXEN0);					 //enable TX
	UCSR0B |= (1 << RXEN0);					 //enable RX
	UCSR0B |= (1 << RXCIE0);				 //RX complete interrupt
	UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00); //no parity, 1 stop bit, 8-bit data
	UCSR0A |= (1 << U2X0);					 //USART Double Transmission Speed
}

/************************************************************************/
/* Method to send single char to RF module */         
/* @param a - char to send	               */
/************************************************************************/
uint8_t sendChar_RFM(uint8_t a)
{
	while (!(UCSR0A & (1 << UDRE0))) //wait for empty transmit buffer
	{
	};
	UDR0 = a;						 //set char to transmit buffer
	return 1;					
}

/************************************************************************/
/* Method to receive single char from RF module */
/* @return a - received char               */
/************************************************************************/
uint8_t receiveChar_RFM(void)
{
	uint8_t a;	//local variable
	while (!(UCSR0A & (1 << RXC0)))  //wait for data in received buffer
	{
	};
	a = UDR0;
	//USART_send(a);  //echo
	return a;
}

/************************************************************************/
/* Method parse received data about cameras from ring buffer            */
/************************************************************************/
uint8_t parseCameraStatus()
{
	int liveCamera;
	getItem(&RX_ringBuffer, &liveCamera);   //get data from ring buffer and move to next

	int readyCamera;
	getItem(&RX_ringBuffer, &readyCamera);  //get data from ring buffer and move to next

	//Parse what camera is in LIVE status from received data
	if (!(liveCamera & CAMERA_MASK))  //first control for actual camera
	{
		liveCameras = CAMERA;		  //set number description of LIVE camera
	}
	else if (!(liveCamera & 0x01))
	{
		liveCameras = 1;
	}
	else if (!(liveCamera & 0x02))
	{
		liveCameras = 2;
	}
	else if (!(liveCamera & 0x04))
	{
		liveCameras = 3;
	}
	else if (!(liveCamera & 0x08))
	{
		liveCameras = 4;
	}
	else if (!(liveCamera & 0x10))
	{
		liveCameras = 5;
	}
	else if (!(liveCamera & 0x20))
	{
		liveCameras = 6;
	}
	else
	{
		liveCameras = 0;					//default status
	}

	//Parse what camera is in READY status from received data
	if (!(readyCamera & CAMERA_MASK))		//first control for actual camera
	{
		readyCameras = CAMERA;				//set number description of READY camera
	}
	else if (!(readyCamera & 0x01))
	{
		readyCameras = 1;
	}
	else if (!(readyCamera & 0x02))
	{
		readyCameras = 2;
	}
	else if (!(readyCamera & 0x04))
	{
		readyCameras = 3;
	}
	else if (!(readyCamera & 0x08))
	{
		readyCameras = 4;
	}
	else if (!(readyCamera & 0x10))
	{
		readyCameras = 5;
	}
	else if (!(readyCamera & 0x20))
	{
		readyCameras = 6;
	}
	else
	{
		readyCameras = 0;			   //default status
	}

	int temp;
	getItem(&RX_ringBuffer, &temp);	   //get last char of message
	
	if(temp == RFM_END_CHAR)		   //check on compliance
	{
		return 1;
	}
	else{
		return 0;
	}
}

/************************************************************************/
/* Method refresh LEDs according actual status            */
/************************************************************************/
void refresh_LED()
{
	if (liveCameras == CAMERA) //camera in LIVE state
	{
		sbi(LED_PORT, R_LED);  //set RED LED to HIGH - turn on
	}
	else
	{
		cbi(LED_PORT, R_LED);  //set RED LED to LOW - turn off

		if (readyCameras == CAMERA)//camera in READY state
		{
			sbi(LED_PORT, G_LED);  //set GREEN LED to HIGH - turn on
		}
		else
		{
			cbi(LED_PORT, G_LED);  //set GREEN LED to LOW - turn off
		}
	}
}

/************************************************************************/
/* Method show on display Tally status as a number */                                                                  
/************************************************************************/
void display_cameraState()
{
	printTallyNumber(liveCameras, 0, 0); //the biggest number for LIVE camera
	printBigNumber(readyCameras, 0, 6);  //ready camera
	printBigNumber(CAMERA, 4, 13);		 //number of actual camera
}

/************************************************************************/
/* Method show on display prepared message   */                                                               
/************************************************************************/
uint8_t display_preparedMessage()
{
	int cameraID;
	int messageID;
	int lastChar;

	getItem(&RX_ringBuffer, &cameraID);		//get data from ring buffer and move to next
	getItem(&RX_ringBuffer, &messageID);
	getItem(&RX_ringBuffer, &lastChar);

	if (cameraID & CAMERA_MASK)				//is message for this camera
	{
		showPreparedMessage(messageID);		//show prepared message	
	}
	
	if(lastChar == RFM_END_CHAR)		   //check on compliance 
	{
		return 1;
	}
	else{
		return 0;
	}
}

/************************************************************************/
/* Method process received data and show message on display   */
/************************************************************************/
void display_writtenMessage()
{
	int cameraID;
	getItem(&RX_ringBuffer, &cameraID);		//read data from ring buffer and move to next

	int temp;
	getItem(&RX_ringBuffer, &temp);

	if (cameraID & CAMERA_MASK)				//is message for this camera
	{
		display_turnOff();
		dispaly_clear();

		uint8_t posX = 0;
		setXY(posX, 0);					     //set X - line, Y - row  position on display

		uint8_t dispalyCharIndex = 0;		 //number of char in row

		while (temp != RFM_END_CHAR)		 //read data until find end
		{
			if (!(dispalyCharIndex < 15))	 //after 16 chars, go to next line
			{
				posX++;
				setXY(posX, 0);				 //set cursor to next line
				dispalyCharIndex = 0;
			}

			getItem(&RX_ringBuffer, &temp);
			display_sendChar(temp);			 //send char to show on display
			dispalyCharIndex++;
		}

		display_turnOn();
	}
	else
	{
		while (temp != RFM_END_CHAR)			//find end of message
		{
			getItem(&RX_ringBuffer, &temp);
		}
	}
}

/************************************************************************/
/* Method finding end of unknown message  */
/************************************************************************/
void unknownMessenge()
{
	int temp;
	do
	{
		getItem(&RX_ringBuffer, &temp);
	} while (temp != RFM_END_CHAR);					//find end of message

}

/************************************************************************/
/* MAIN                                                                 */
/************************************************************************/
int main(void)
{
	initializeQueue(&RX_ringBuffer);  //Initialization ring buffer for received data from RF module
	
	init_display();
	display_cameraState();
	
	init_LED();
	init_RFM_UART();
	init_button();
	init_T0_WD();
	init_T2_autoDisplayCleaner();

	sei(); //enable global interrupt

	int messageStatus;

	//main infinite loop
	while (1)
	{
		if (watchdog == false)		  //watchdog - system has fresh data
		{
			if (newMessenges != 0)	  //is available new message
			{
				cbi(LED_PORT, B_LED); //set BLUE LED to LOW - tunr off watchdog

				getItem(&RX_ringBuffer, &messageStatus);

				switch (messageStatus) //whats type of received message
				{

				case REFRESH: 

				case CHANGED:
					parseCameraStatus();
					refresh_LED(); 
					if (textOnDisplay == false) //is on display text message
					{
						display_cameraState();
					}

					break;

				case MESSAGE_BASIC: 
					display_preparedMessage();
					textOnDisplay = true;		//message is showed on display
					timerx_T2 = 0;				//reset timer variable

					break;

				case MESSAGE_ADVANCED: 
					display_writtenMessage();
					textOnDisplay = true;		//message is showed on display
					timerx_T2 = 0;				//reset timer variable

					break;

				case RESPONSE: 
					
				default: 
					unknownMessenge();			//Process unknown massage
					break;
				}
				newMessenges--;					//decrementation number of new unprocessed message
			}
		}
		else   // warchdow is on
		{
			cbi(LED_PORT, R_LED);	//turn off RED LED 
			cbi(LED_PORT, G_LED);	//turn off GREEN LED 

			tbi(LED_PORT, B_LED);	//change logical level of BLUE LED
			_delay_ms(WATCHDOG_ERROR_TIME);
		}

		if (systemError == true)	//Fatal system error
		{
			cli();					//disable global interrupt
			while (1)				//infinity loop
			{
				tbi(LED_PORT, R_LED); //change logical level of RED LED
				_delay_ms(FATAL_ERROR_TIME);
			}
		}

		_delay_ms(MAIN_LOOP_TIME); 
	}
}

/********************INTERRUPTS*********************/

/************************************************************************/
/* Interrupt handling of USART - after end of UART communication        */
/************************************************************************/
ISR(USART_RX_vect)
{
	watchdog = false;	 // turn off watchdog
	timerx_T0 = 0;		 // reset watchdog timer variable

	uint8_t receivedChar = UDR0;  //get received char from hardware buffer

	if (receivedChar == RFM_END_CHAR)	//test end of message
	{
		newMessenges++;					//increment count of new message
	}

	putItem(&RX_ringBuffer, receivedChar); //put received data to ring buffer
}

/************************************************************************/
/* Interrupt handling for TIMER 0 - after reach compared value          */
/************************************************************************/
ISR(TIMER0_COMPA_vect)
{
	timerx_T0++;						//increment addition variable
	if (timerx_T0 >= WATCHDOG_ISR_CNT)	//procedure for reach 10 second timer interval
	{									//main part of interrupt
		watchdog = true;				//turn on watchdog
		timerx_T0 = 0;					//reset variable
	}
}

/************************************************************************/
/* Interrupt handling for TIMER 1 - after reach compared value          */
/************************************************************************/
ISR(TIMER2_COMPA_vect) 
{
	if (textOnDisplay == true)			//is text showed on display
	{
		timerx_T2++;					//increment addition variable
		if (timerx_T2 > SHOWMESSENGE_ISR_CNT) //procedure for reach 10 second timer interval
		{								//main part of interrupt
			textOnDisplay = false;		
			timerx_T2 = 0;				//reset variable
			dispaly_clear();
			display_cameraState();		//show camera status on Display	
		}
	}
}

/************************************************************************/
/* Interrupt handling for PIN CHANGED STATE - after change value        */
/************************************************************************/
ISR(PCINT0_vect) 
{
	uint8_t changedbits;						
	uint8_t intreading = BUTTON_PIN & 0x7;		//read pin status and use mask for first 3 bits
	changedbits = intreading ^ portbHistory;	//XOR operation with last change status - control to press - 1 or release - button
	portbHistory = intreading;					//save last status of port
	
	switch(changedbits){						//What button was pressed

		case 0:									//nothing changed
		break;

		case 1:									//pcint 0 changed
			if(portbHistory & 0x01)				//changed to press
			{
				sendChar_RFM(RESPONSE);
				sendChar_RFM(CAMERA_MASK);
				sendChar_RFM(YES);
				sendChar_RFM(RFM_END_CHAR);
			}
			
		
		break;

		case 2:									//pcint 1 changed
		if(portbHistory & 0x02)					//changed to press
		{
			textOnDisplay = false;
			timerx_T2 = 0;
			dispaly_clear();
			display_cameraState();
		}
		break;

		case 3:									//pcint 0+1 changed
		break;

		case 4:									//pcint 2 changed
		if(portbHistory & 0x04)					//changed to press
		{
		sendChar_RFM(RESPONSE);
		sendChar_RFM(CAMERA_MASK);
		sendChar_RFM(NO);
		sendChar_RFM(RFM_END_CHAR);
		}
		
		break;

		case 5:									//pcint 2+0 changed
		break;

		case 6:									//pcint 2+1
		break;

		case 7:									//pcint all changed

		break;
}

}