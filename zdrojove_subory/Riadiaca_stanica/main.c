/*
 * MainStation.c
 *
 * Title: Diploma thesis - Communication for cameraman
 * Created: 6/3/2018 1:47:07 PM
 * Author: Tomáš Klein
 */

#include "board.h"

#include "avr_compiler.h"
#include "usart_driver.h"
#include "port_driver.h"
#include "TC_driver.h"

#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include <avr/io.h>


USART_data_t USART_data_RFM;     // USART data struct used as software data buffer 
USART_data_t USART_data_BLT;
USART_data_t USART_data_PC;

volatile uint8_t port_A;		//last known status of port
volatile uint8_t port_B;   

uint8_t message_cameraStatus[4] = {0x00, 0x00, 0x00, UART_END_CHAR};  //template: messageID, CAMERA_LIVE, CAMERA_READY, end char for message

uint8_t RX_RFM_data[RX_NUM_BYTES];		//RX software buffer for one message
uint8_t RX_RFM_index = 0;				//count of data in buffer

uint8_t RX_PC_data[RX_NUM_BYTES];
uint8_t RX_PC_index = 0;

uint8_t RX_BLT_data[RX_NUM_BYTES];
uint8_t RX_BLT_index = 0;

/************************************************************************/
/* Initialization for connector with switcher, connection by DSUB 15    */
/************************************************************************/
void init_gpiConnector()
{
	PORT_ConfigurePins(&PORTA, 0xFF,false,false, PORT_OPC_PULLUP_gc, PORT_ISC_RISING_gc); //configuration for PORT A 
	PORT_ConfigurePins(&PORTB, 0x0F,false,false, PORT_OPC_PULLUP_gc, PORT_ISC_RISING_gc); //configuration for PORT B
}

/************************************************************************/
/* Initialization of RGB LEDs                                           */
/************************************************************************/
void init_LED()
{
	unsigned char const mask = PIN0_bm | PIN1_bm | PIN2_bm; // We only want to use pin 0, 1, and 2 of the LED port.
	PORTD.DIR = mask;										// Set all pins of port D to output.
	PORTD.OUT = 0x00;										// Set upper four bits high.
}

/************************************************************************/
/* Initialization UART interface for RF module                          */
/* USART configuration: - 8 bit character size                          */
/*                      - No parity                                     */
/*                      - 1 stop bit                                    */
/*                      - 9600 Baud                                     */
/************************************************************************/
void init_RFM_UART()
{
	RFM_PORT.DIRSET = PIN3_bm;   //set RFM TX as output
	RFM_PORT.DIRCLR = PIN2_bm;   //set RFM RX as input

	RFM_PORT.DIRSET = PIN0_bm;	 //set RFM M0 as output
	RFM_PORT.OUTCLR = PIN0_bm;   //set RFM M0 to LOW

	RFM_PORT.DIRSET = PIN1_bm;   //set RFM M1 as output
	RFM_PORT.OUTCLR = PIN1_bm;	 //set RFM M1 to LOW

	USART_InterruptDriver_Initialize(&USART_data_RFM, &RFM_UART, USART_DREINTLVL_LO_gc);  //use USART and initialize buffers
	
	USART_Format_Set(&RFM_UART, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);    //USART configuration: 8 Data bits, No Parity, 1 Stop bit
	
	USART_RxdInterruptLevel_Set(&RFM_UART, USART_RXCINTLVL_LO_gc);  //enable RXC interrupt

	/* Set Baudrate to 9600 bps:
	 * Use the default I/O clock fequency that is 2 MHz.
	 * Do not use the baudrate scale factor
	 *
	 * Baudrate select = (1/(16*(((I/O clock frequency)/Baudrate)-1)
	 *                 = 12
	 */
	USART_Baudrate_Set(&RFM_UART, 12, 0); //set baudrate

	USART_Rx_Enable(&RFM_UART);  //enable RX
	USART_Tx_Enable(&RFM_UART);  //enable TX
}

/************************************************************************/
/* Initialization UART interface for Bluetooth module                   */
/* USART configuration: - 8 bit character size                          */
/*                      - No parity                                     */
/*                      - 1 stop bit                                    */
/*                      - 9600 Baud                                     */
/************************************************************************/
void init_BLT_UART()
{

	BLT_PORT_UART.DIRSET = PIN7_bm;   //set BLT TX as output
	BLT_PORT_UART.DIRCLR = PIN6_bm;   //set BLT RX as input

	PORT_ConfigurePins(&BLT_PORT_UART, 0x20, false, false, PORT_OPC_PULLUP_gc, PORT_ISC_RISING_gc); //set BLT STATUS as input
	
	USART_InterruptDriver_Initialize(&USART_data_BLT, &BLT_USART, USART_DREINTLVL_LO_gc);	//use USART and initialize buffers

	USART_Format_Set(&BLT_USART, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false); //USART configuration: 8 Data bits, No Parity, 1 Stop bit

	USART_RxdInterruptLevel_Set(&BLT_USART, USART_RXCINTLVL_LO_gc);   //enable RXC interrupt

	USART_Baudrate_Set(&BLT_USART, 12, 0);   //set baudrate

	USART_Rx_Enable(&BLT_USART);   //enable RX
	USART_Tx_Enable(&BLT_USART);   //enable TX
}

/************************************************************************/
/* Initialization UART interface for PC debug                           */
/* USART configuration: - 8 bit character size                          */
/*                      - No parity                                     */
/*                      - 1 stop bit                                    */
/*                      - 9600 Baud                                     */
/************************************************************************/
void setUp_PC_USART()
{

	USART_PORT_PC.DIRSET = PIN3_bm;		//set BLT TX as output
	USART_PORT_PC.DIRCLR = PIN2_bm;		//set BLT RX as input

	USART_InterruptDriver_Initialize(&USART_data_PC, &USART_PC, USART_DREINTLVL_LO_gc); //use USART and initialize buffers

	USART_Format_Set(&USART_PC, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false); //USART configuration: 8 Data bits, No Parity, 1 Stop bit

	USART_RxdInterruptLevel_Set(&USART_PC, USART_RXCINTLVL_LO_gc); //enable RXC interrupt

	USART_Baudrate_Set(&USART_PC, 12, 0); //set baudrate

	USART_Rx_Enable(&USART_PC);		//enable RX
	USART_Tx_Enable(&USART_PC);		//enable TX
}

/************************************************************************/
/* Initialization Timer for refresh messages - 4 second interval        */
/************************************************************************/
void init_beaconMessageTimer()
{
	TC_SetPeriod(&TCC0, BEACON_REFRESH);				//set the TC period 
	TC0_SetOverflowIntLevel(&TCC0, TC_OVFINTLVL_LO_gc); //enable overflow interrupt at low level
	TC0_ConfigClockSource(&TCC0, TC_CLKSEL_DIV1024_gc);	//start Timer/Counter
}

/************************************************************************/
/* Initialization Timer for control GPI connector - 200 ms interval     */
/************************************************************************/
void init_gpiControlTimer()
{
	TC_SetPeriod(&TCC1, OUTPUT_PIN_REFRESH);			//set the TC period 
	TC1_SetOverflowIntLevel(&TCC1, TC_OVFINTLVL_LO_gc);	//enable overflow interrupt at low level
	TC1_ConfigClockSource(&TCC1, TC_CLKSEL_DIV1024_gc); //start Timer/Counter
}

/************************************************************************/
/* Method to send chat via RFM UART communication				        */
/************************************************************************/
void RFM_sendChar(char c)
{
	while (!(USARTC0_STATUS & USART_DREIF_bm))  ; //Wait until DATA buffer is empty
	USARTC0_DATA = c;		//put data to buffer
}

/************************************************************************/
/* Method to send string via RFM UART communication				        */
/************************************************************************/
void RFM_sendString(char *text)
{
	while (*text)
	{
		RFM_sendChar(*text++);
	}
}

/************************************************************************/
/* Forward message from Bluetooth module to RF communication module     */
/************************************************************************/
void sendBuffer_fromBleutoothToRfm(void)
{
	TC_Restart(&TCC0);  //Restart timer for refresh messages

	uint8_t i = 0;
	bool byteToBuffer;
	
	while (i <= RX_BLT_index)	//Send all data from Bluetooth buffer
	{	
		byteToBuffer = USART_TXBuffer_PutByte(&USART_data_RFM, RX_BLT_data[i]); //put data to buffer and get action status
		if (byteToBuffer)		//Validation to correct sending
		{
			i++;				//go to next char
		}
	}
	RX_BLT_index = 0;			//reset Bluetooh buffer
}

/************************************************************************/
/* Forward message from RF communication module to Bluetooth module     */
/************************************************************************/
void sendBuffer_fromRfmToBleutooth(void)
{
	uint8_t i = 0;
	bool byteToBuffer;
	
	while (i <= RX_RFM_index)	//Send all data from RFM buffer
	{
		byteToBuffer = USART_TXBuffer_PutByte(&USART_data_BLT, RX_RFM_data[i]); //put data to buffer and get action status
		if (byteToBuffer)		//Validation to correct sending
		{
			i++;				//go to next char
		}
	}
	RX_RFM_index = 0;			//reset Bluetooh buffer
}
/************************************************************************/
/* Send REFRESH/CHANGE message to signalization unit via RF module   	*/
/************************************************************************/
void sendMessage_RFM(void)
{
	TC_Restart(&TCC0);			//Restart timer for refresh messages
	
	uint8_t	i = 0;
	bool byteToBuffer;
	
	while (i < 4) {				//send message_cameraStatus
		byteToBuffer = USART_TXBuffer_PutByte(&USART_data_RFM, message_cameraStatus[i]); //put data to buffer and get action status
		if(byteToBuffer){
			i++;				//go to next char
		}
	}
}

/************************************************************************/
/* Main                                                                 */
/************************************************************************/
int main(void)
{		
	//Initialization
	init_RFM_UART();
	init_BLT_UART();
	init_beaconMessageTimer();
	init_gpiControlTimer();
	
	// Enable all interrupt levels in PMIC and enable global interrupts.
	PMIC.CTRL |= PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
	sei();
	
	//Main infinity loop
	while (1)
	{
		//Send Tally camera status or refresh message
		if(message_cameraStatus[0] != NORMAL)  //Control to changed status - REFRESH or CHANGED
		{
			sendMessage_RFM();				   //send status to cameraman
			message_cameraStatus[0] = NORMAL;  //set system status to NORMAL
		}

		//Forward BLT received data to RF communication module
		while (USART_RXBufferData_Available(&USART_data_BLT)) //Process all available received data from Bluetooth module
		{
			RX_BLT_data[RX_BLT_index] = USART_RXBuffer_GetByte(&USART_data_BLT); //save data in software buffer
			if (RX_BLT_data[RX_BLT_index] == UART_END_CHAR)  //test received data to end of message
			{
				sendBuffer_fromBleutoothToRfm();			//forward message to RFM module
			}
			else
			{
				RX_BLT_index++;				
				if(RX_BLT_index >= RX_NUM_BYTES) //control to overflow buffer
				{
					RX_BLT_index = 0;
				}						
			}
		}
	
		//Forward RFM received data to Bluetooth module
		while (USART_RXBufferData_Available(&USART_data_RFM))	//Process all available received data from RFM module
		{
			RX_RFM_data[RX_RFM_index] = USART_RXBuffer_GetByte(&USART_data_RFM); //save data in software buffer
			if (RX_RFM_data[RX_RFM_index] == UART_END_CHAR)		//test received data to end of message
			{
				sendBuffer_fromRfmToBleutooth();				//forward message to Bluetooth module
			}
			else
			{
				RX_RFM_index++;
				if(RX_BLT_index >= RX_NUM_BYTES)				//control to overflow buffer
				{
					RX_BLT_index = 0;
				}
			}
		}

		_delay_ms(SYSTEM_REFRESH);	   	 
	}
}


/********************INTERRUPTS*********************/

/************************************************************************/
/* UART RFM - Receive complete interrupt service routine				*/
/************************************************************************/
ISR(USARTC0_RXC_vect)
{
	USART_RXComplete(&USART_data_RFM);	//move received data from hardware buffer to software buffer
}

/************************************************************************/
/* UART PC - Receive complete interrupt service routine					*/
/************************************************************************/
ISR(USARTE0_RXC_vect)
{
	USART_RXComplete(&USART_data_PC);  //move received data from hardware buffer to software buffer
}

/************************************************************************/
/* UART BLT - Receive complete interrupt service routine			    */
/************************************************************************/
ISR(USARTC1_RXC_vect)
{
	USART_RXComplete(&USART_data_BLT);  //move received data from hardware buffer to software buffer
}

/************************************************************************/
/* UART RFM - Data register empty interrupt service routine				*/
/************************************************************************/
ISR(USARTC0_DRE_vect)
{
	USART_DataRegEmpty(&USART_data_RFM);
}

/************************************************************************/
/* UART PC - Data register empty interrupt service routine				*/
/************************************************************************/
ISR(USARTE0_DRE_vect)
{
	USART_DataRegEmpty(&USART_data_PC);
}

/************************************************************************/
/* UART BLT - Data register empty interrupt service routine			    */
/************************************************************************/
ISR(USARTC1_DRE_vect)
{
	USART_DataRegEmpty(&USART_data_BLT);
}

/************************************************************************/
/* Interrupt handling for TIMER 1 - check GPI connector changed - TALLY */
/************************************************************************/
ISR(TCC1_OVF_vect) // TALLY
{
	uint8_t port_A_temp = PORT_GetPortValue(&PORTA);	//read value on port
	uint8_t port_B_temp = PORT_GetPortValue(&PORTB);	//read value on port

	if (port_A_temp != port_A || port_B_temp != port_B) //compare to change value
	{
		port_A = port_A_temp;		//save actual state of port
		port_B = port_B_temp;		//save actual state of port

		message_cameraStatus[1] = ((port_A & 0xC0) >> 2) + port_B; 	//set LIVE camera to message with mask - last 2 bits from port A and first 4 bits from port B
		message_cameraStatus[2] = port_A & 0x3F;					//set READY camera to message with mask - just first six port A are for READY signal
		
		message_cameraStatus[0] = CHANGED; //set message/system status
	}
}

/************************************************************************/
/* Interrupt handling for TIMER 0 - send refresh message			    */
/************************************************************************/
ISR(TCC0_OVF_vect) 
{
	message_cameraStatus[0] = REFRESH; //set message/system status
}