/*
 * MainStation.c
 *
 * Created: 12/22/2018 10:14:10 AM
 * Author : klein
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

//USART riesim za pomoci dodavanych zdrojakov -projekt AVR1522 - 2 task

/* USART data struct used in task */
USART_data_t USART_data_RFM;

/* USART data struct used in task */
USART_data_t USART_data_BLT;

/* USART data struct used in task */
USART_data_t USART_data_PC;



volatile	uint8_t port_A;   //Ulozena posledna hodnota na porte A
volatile	uint8_t port_B;	  //Ulozena posledna hodnota na porte B


// Dovod spravy, CAMERA_LIVE, CAMERA_READY, Ukoncovaci znak spravy
uint8_t cameraStatus[4] = {0x00, 0x00, 0x00, USART_END_CHAR}; 




uint8_t RX_RFM_data[RX_NUM_BYTES];
uint8_t RX_RFM_index = 0;

uint8_t RX_PC_data[RX_NUM_BYTES];
uint8_t RX_PC_index = 0;

uint8_t RX_BLT_data[RX_NUM_BYTES];
uint8_t RX_BLT_index = 0;

bool newStatus = false;
bool newMessenge = false;
bool newResponse = false;

void setTally_CONN()
{
	PORT_ConfigurePins(&PORTA,
					   0xFF,
					   false,
					   false,
					   PORT_OPC_PULLUP_gc,
					   PORT_ISC_RISING_gc);

	PORT_ConfigurePins(&PORTB,
					   0x0F,
					   false,
					   false,
					   PORT_OPC_PULLUP_gc,
					   PORT_ISC_RISING_gc);
}

void setLED()
{
	unsigned char const mask = PIN0_bm | PIN1_bm | PIN2_bm; // We only want to use pin 0, 1, and 2 of the LED port.
	PORTD.DIR = mask;    // Set all pins of port D to output.
	PORTD.OUT = 0x00;    // Set upper four bits high.
}

void setUp_RFM_USART()
{
	/*
	 *    USART configuration:
	 *      - 8 bit character size
	 *      - No parity
	 *      - 1 stop bit
	 *      - 9600 Baud
	 */

	/* PC3 (TXD0) as output. */
	USART_PORT_LORA.DIRSET = PIN3_bm;

	/* PC2 (RXD0) as input. */
	USART_PORT_LORA.DIRCLR = PIN2_bm;

	/* M0 - nastavenie na nulu */
	USART_PORT_LORA.DIRSET = PIN0_bm;
	USART_PORT_LORA.OUTCLR = PIN0_bm;

	/* M1 - nastavenie na nulu */
	USART_PORT_LORA.DIRSET = PIN1_bm;
	USART_PORT_LORA.OUTCLR = PIN1_bm;

	/* Use USARTx0 and initialize buffers */
	USART_InterruptDriver_Initialize(&USART_data_RFM, &USART_LORA, USART_DREINTLVL_LO_gc);

	/* USART, 8 Data bits, No Parity, 1 Stop bit. */
	USART_Format_Set(&USART_LORA, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);

	/* Enable RXC interrupt */
	//USART_RxdInterruptLevel_Set(USART_data_RFM.usart, USART_RXCINTLVL_LO_gc);
	USART_RxdInterruptLevel_Set(&USART_LORA, USART_RXCINTLVL_LO_gc);

	/* Set Baudrate to 9600 bps:
	 * Use the default I/O clock fequency that is 2 MHz.
	 * Do not use the baudrate scale factor
	 *
	 * Baudrate select = (1/(16*(((I/O clock frequency)/Baudrate)-1)
	 *                 = 12
	 */
	USART_Baudrate_Set(&USART_LORA, 12, 0);

	/* Enable both RX and TX. */
	USART_Rx_Enable(&USART_LORA);
	USART_Tx_Enable(&USART_LORA);
}

void setUp_BLT_USART()
{
	/*
	 *    USART configuration:
	 *      - 8 bit character size
	 *      - No parity
	 *      - 1 stop bit
	 *      - 9600 Baud
	 */

	/* PC7 (TXD1) as output. */
	USART_PORT_BLT.DIRSET = PIN7_bm;

	/* PC6 (RXD1) as input. */
	USART_PORT_BLT.DIRCLR = PIN6_bm;

	//Nastavovanie vstupu STATUS z BLT
	PORT_ConfigurePins(&USART_PORT_BLT,
					   0x20,
					   false,
					   false,
					   PORT_OPC_PULLUP_gc,
					   PORT_ISC_RISING_gc);

	
	/* Use USARTx0 and initialize buffers */
	USART_InterruptDriver_Initialize(&USART_data_BLT, &USART_BLT, USART_DREINTLVL_LO_gc);

	/* USART, 8 Data bits, No Parity, 1 Stop bit. */
	USART_Format_Set(&USART_BLT, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);

	/* Enable RXC interrupt */
	//USART_RxdInterruptLevel_Set(USART_data_RFM.usart, USART_RXCINTLVL_LO_gc);
	USART_RxdInterruptLevel_Set(&USART_BLT, USART_RXCINTLVL_LO_gc);

	/* Set Baudrate to 9600 bps:
	 * Use the default I/O clock fequency that is 2 MHz.
	 * Do not use the baudrate scale factor
	 *
	 * Baudrate select = (1/(16*(((I/O clock frequency)/Baudrate)-1)
	 *                 = 12
	 */
	USART_Baudrate_Set(&USART_BLT, 12, 0);

	/* Enable both RX and TX. */
	USART_Rx_Enable(&USART_BLT);
	USART_Tx_Enable(&USART_BLT);
}

void setUp_PC_USART()
{
	/*
	 *    USART configuration:
	 *      - 8 bit character size
	 *      - No parity
	 *      - 1 stop bit
	 *      - 9600 Baud
	 */

	/* PE3 (TXD0) as output. */
	USART_PORT_PC.DIRSET = PIN3_bm;

	/* PE2 (RXD0) as input. */
	USART_PORT_PC.DIRCLR = PIN2_bm;

	/* Use USARTx0 and initialize buffers */
	USART_InterruptDriver_Initialize(&USART_data_PC, &USART_PC, USART_DREINTLVL_LO_gc);

	/* USART, 8 Data bits, No Parity, 1 Stop bit. */
	USART_Format_Set(&USART_PC, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);

	/* Enable RXC interrupt */
	//USART_RxdInterruptLevel_Set(USART_data_RFM.usart, USART_RXCINTLVL_LO_gc);
	USART_RxdInterruptLevel_Set(&USART_PC, USART_RXCINTLVL_LO_gc);

	/* Set Baudrate to 9600 bps:
	 * Use the default I/O clock fequency that is 2 MHz.
	 * Do not use the baudrate scale factor
	 *
	 * Baudrate select = (1/(16*(((I/O clock frequency)/Baudrate)-1)
	 *                 = 12
	 */
	USART_Baudrate_Set(&USART_PC, 12, 0);

	/* Enable both RX and TX. */
	USART_Rx_Enable(&USART_PC);
	USART_Tx_Enable(&USART_PC);
}

void setTimer_Beacon()
{
	/* Set the TC period. */
	TC_SetPeriod(&TCC0, BEACON_REFRESH); //5000ms

	/* Enable overflow interrupt at low level */
	TC0_SetOverflowIntLevel(&TCC0, TC_OVFINTLVL_LO_gc);

	/* Start Timer/Counter. */
	TC0_ConfigClockSource(&TCC0, TC_CLKSEL_DIV1024_gc);
}

void setTimer_Tally()
{
	/* Set the TC period. */
	TC_SetPeriod(&TCC1, OUTPUT_PIN_REFRESH); //200ms

	/* Enable overflow interrupt at low level */
	TC1_SetOverflowIntLevel(&TCC1, TC_OVFINTLVL_LO_gc);

	/* Start Timer/Counter. */
	TC1_ConfigClockSource(&TCC1, TC_CLKSEL_DIV1024_gc);
}


void sendCharRFM(char c)
{

	while (!(USARTC0_STATUS & USART_DREIF_bm))
		; //Wait until DATA buffer is empty

	USARTC0_DATA = c;
	
}

void sendStringRFM(char *text)
{
	while (*text)
	{
		sendCharRFM(*text++);
	}
}

void sendBLT2RFM(void)
{
	TC_Restart(&TCC0);

	uint8_t i = 0;
	while (i <= RX_BLT_index)
	{
		bool byteToBuffer;
		byteToBuffer = USART_TXBuffer_PutByte(&USART_data_RFM, RX_BLT_data[i]);
		if (byteToBuffer)
		{
			i++;
		}
	}
	RX_BLT_index = 0;
}

void sendRFM2BLT(void)
{

	uint8_t i = 0;
	while (i <= RX_RFM_index)
	{
		bool byteToBuffer;
		byteToBuffer = USART_TXBuffer_PutByte(&USART_data_BLT, RX_RFM_data[i]);
		if (byteToBuffer)
		{
			i++;
		}
	}
	RX_RFM_index = 0;
}

void sendSTATUS2RFM(void)
{
	TC_Restart(&TCC0);
	
	uint8_t	i = 0;
	while (i < 4) {
		bool byteToBuffer;
		byteToBuffer = USART_TXBuffer_PutByte(&USART_data_RFM, cameraStatus[i]);
		if(byteToBuffer){
			i++;
		}
	}
}

int main(void)
{
	//setLED();	
		
	setUp_RFM_USART();

	//setUp_PC_USART();
	
	setUp_BLT_USART();

	setTimer_Beacon();

	setTimer_Tally();
	

	// Enable all interrupt levels in PMIC and enable global interrupts.
	PMIC.CTRL |= PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
	sei();

	while (1)
	{

		if(cameraStatus[0] != NORMAL)
		{
			sendSTATUS2RFM();
			cameraStatus[0] = NORMAL;
		}

		//Preposielanie Bluetooth prijatych dat na RF komunikacny kanal
		while (USART_RXBufferData_Available(&USART_data_BLT))
		{
			RX_BLT_data[RX_BLT_index] = USART_RXBuffer_GetByte(&USART_data_BLT);
			if (RX_BLT_data[RX_BLT_index] == USART_END_CHAR)
			{
				sendBLT2RFM();
			}
			else
			{
				RX_BLT_index++;
			}
		}
	
		//Preposielanie RF komunikacie na BLT
		while (USART_RXBufferData_Available(&USART_data_RFM))
		{
			RX_RFM_data[RX_RFM_index] = USART_RXBuffer_GetByte(&USART_data_RFM);
			if (RX_RFM_data[RX_RFM_index] == USART_END_CHAR)
			{
				sendRFM2BLT();
			}
			else
			{
				RX_BLT_index++;
			}
		}

		_delay_ms(SYSTEM_REFRESH);	   	 
	}
}

/*! \brief Receive complete interrupt service routine.
 *
 *  Receive complete interrupt service routine.
 *  Calls the common receive complete handler with pointer to the correct USART
 *  as argument.
 */
ISR(USARTC0_RXC_vect)
{
	USART_RXComplete(&USART_data_RFM);
}

ISR(USARTE0_RXC_vect)
{
	USART_RXComplete(&USART_data_PC);
}

ISR(USARTC1_RXC_vect)
{
	USART_RXComplete(&USART_data_BLT);
}


/*! \brief Data register empty  interrupt service routine.
 *
 *  Data register empty  interrupt service routine.
 *  Calls the common data register empty complete handler with pointer to the
 *  correct USART as argument.
 */
ISR(USARTC0_DRE_vect)
{
	USART_DataRegEmpty(&USART_data_RFM);
}

ISR(USARTE0_DRE_vect)
{
	USART_DataRegEmpty(&USART_data_PC);
}

ISR(USARTC1_DRE_vect)
{
	USART_DataRegEmpty(&USART_data_BLT);
}


// Tell compiler to associate this interrupt handler with the TCC0_OVF_vect vector.
ISR(TCC1_OVF_vect) // TALLY
{
	uint8_t port_A_temp = PORT_GetPortValue(&PORTA);
	uint8_t port_B_temp = PORT_GetPortValue(&PORTB);

	if (port_A_temp != port_A || port_B_temp != port_B)
	{
		port_A = port_A_temp;
		port_B = port_B_temp;

		cameraStatus[1] = ((port_A & 0xC0) >> 2) + port_B; 	//LIVE CAMERA
		cameraStatus[2] = port_A & 0x3F;					//READY CAMERA

		cameraStatus[0] = CHANGED; //Odoslanie dat pri zmene na portoch
	}
}

ISR(TCC0_OVF_vect) // BEACON
{
	cameraStatus[0] = REFRESH; //Pravidelne zasielanie dat
}