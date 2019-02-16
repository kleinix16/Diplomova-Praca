/*
* cv1.c
*
* Created: 10/10/2017 3:49:38 PM
* Author :
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

circularQueue_t myQueue;

uint8_t new_msg = 0;

volatile uint8_t portbhistory = 0xFF;     // default is high because the pull-up

bool system_error = false;
bool watchdog = false;

bool zobrazenaSprava = false;

volatile uint8_t CAM_READY = 10; // Cislo kamery v pripravnom rezime
volatile uint8_t CAM_LIVE = 10;  // Cislo kamery v ostrom vyslieani

uint16_t timerx_T0;

uint16_t timerx_T2;

void initBuffer()
{

	initializeQueue(&myQueue);
	for (int i = 0; i < MAX_ITEMS + 1; i++)
	{
		putItem(&myQueue, i);
	}
}

void setup_LED(void)
{
	sbi(LED_DDR, G_LED); //Nastavenie pinu ako vystup
	sbi(LED_DDR, R_LED);
	sbi(LED_DDR, B_LED);

	sbi(LED_PORT, G_LED); //Nastavenie pociatocnej hodnoty - LEDky nestivtia
	sbi(LED_PORT, R_LED);
	sbi(LED_PORT, B_LED);

//	sbi(DDRB, B_LED); //Modra an testovacej doske
//	cbi(PORTB, B_LED);
}

void setup_BUTTON(void)
{
	sbi(BUTTON_PORT, BT_1); //Nastavovanie Pull-Up rezistorov
	sbi(BUTTON_PORT, BT_2);
	sbi(BUTTON_PORT, BT_3);

	cbi(BUTTON_DDR, BT_1); //Nastavenie pinu ako vstup
	cbi(BUTTON_DDR, BT_2);
	cbi(BUTTON_DDR, BT_3);
	
	sbi(PCICR, PCIE0); // Pin Change Interrupt enable on PCINT0 (PB0)
	
	sbi(PCMSK0, PCINT0);
	sbi(PCMSK0, PCINT1);
	sbi(PCMSK0, PCINT2);
	
	
	//PCICR |= _BV(PCIE0);
	//PCMSK0 |= _BV(PCINT0);
	
	
}

void setup_T0_WD()
{
	TCCR0A = 0x2; //rezim CTC interrupt pri zhode z OCR0A
	TCCR0B = 0x5; //clk/1024
	OCR0A = WATCHDOG_ISR_CMP;
	TIMSK0 |= (1 << OCIE0A); //lokalme povolenie prerusenia
}

void setup_T1_showedMess()
{
	TCCR2A = 0x2; //rezim CTC interrupt pri zhode z OCR0A
	TCCR2B = 0x5; //clk/1024
	OCR2A = SHOWMESSENGE_ISR_CMP;
	TIMSK2 |= (1 << OCIE2A); //lokalme povolenie prerusenia
}

void setup_Display(void)
{
	I2C_Init();

	_delay_ms(10);
	InitializeDisplay();

	clear_display();
}

void setup_USART(void)
{
	//Nastavovanie AUX portu ako vstup

	//Nastavovanie M0 a M1 ako vystup s urovnou 0
	DDRD |= (1 << RFM_M0) | (1 << RFM_M1);
	PORTD &= ~(1 << RFM_M0) & ~(1 << RFM_M1);

	//Nastavovanie RX a TX
	DDRD |= (1 << USART_TX); // Tx output

	DDRD &= ~(1 << USART_RX); // Rx input
	PORTD |= (1 << USART_RX); // RX - Pull Up rezistor

	UBRR0H = (uint8_t)(UBRR_VALUE >> 8); // BAUDERATE
	UBRR0L = (uint8_t)UBRR_VALUE;

	UCSR0B |= (1 << TXEN0);					 //enable TX
	UCSR0B |= (1 << RXEN0);					 //enable RX
	UCSR0B |= (1 << RXCIE0);				 //RX complete interrupt
	UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00); // no parity, 1 stop bit, 8-bit data
	UCSR0A |= (1 << U2X0);					 //USART Double Transmission Speed
}

uint8_t USART_send(uint8_t a)
{
	while (!(UCSR0A & (1 << UDRE0)))
	{
	};
	UDR0 = a;
	return 0;
}

uint8_t USART_receive(void)
{
	uint8_t a;
	while (!(UCSR0A & (1 << RXC0)))
	{
	};
	a = UDR0;
	//USART_send(a);  //echo
	return a;
}

void parseCameraStatus()
{
	int liveCamera;
	getItem(&myQueue, &liveCamera);

	int readyCamera;
	getItem(&myQueue, &readyCamera);

	// Ako prve skontrolujem stav tejto kamery - LIVE rezim
	if (!(liveCamera & CAMERA_MASK))
	{
		CAM_LIVE = CAMERA;
	}
	else if (!(liveCamera & 0x01))
	{
		CAM_LIVE = 1;
	}
	else if (!(liveCamera & 0x02))
	{
		CAM_LIVE = 2;
	}
	else if (!(liveCamera & 0x04))
	{
		CAM_LIVE = 3;
	}
	else if (!(liveCamera & 0x08))
	{
		CAM_LIVE = 4;
	}
	else if (!(liveCamera & 0x10))
	{
		CAM_LIVE = 5;
	}
	else if (!(liveCamera & 0x20))
	{
		CAM_LIVE = 6;
	}
	else
	{
		CAM_LIVE = 0;
	}

	// Ako prve skontrolujem stav tejto kamery - READY rezim
	if (!(readyCamera & CAMERA_MASK))
	{
		CAM_READY = CAMERA;
	}
	else if (!(readyCamera & 0x01))
	{
		CAM_READY = 1;
	}
	else if (!(readyCamera & 0x02))
	{
		CAM_READY = 2;
	}
	else if (!(readyCamera & 0x04))
	{
		CAM_READY = 3;
	}
	else if (!(readyCamera & 0x08))
	{
		CAM_READY = 4;
	}
	else if (!(readyCamera & 0x10))
	{
		CAM_READY = 5;
	}
	else if (!(readyCamera & 0x20))
	{
		CAM_READY = 6;
	}
	else
	{
		CAM_READY = 0;
	}

	int pomChar;
	getItem(&myQueue, &pomChar); //Nacitanie koncoveho znaku USART
}

void refresh_LED()
{
	//Kontrola, ci je kamera LIVE
	if (CAM_LIVE == CAMERA) //Aplikovanie masky pre zistenie stavu kamerz
	{
		sbi(LED_PORT, R_LED); //Rozsvietenie cervenej LED
	}
	else
	{
		cbi(LED_PORT, R_LED); //Zhsnutie cervenej LED

		//Kontrola, ci je kamera READY
		if (CAM_READY == CAMERA)
		{
			sbi(LED_PORT, G_LED); //Rozsvietenie zelenej
		}
		else
		{
			cbi(LED_PORT, G_LED); //Zhasnutie zelenej
		}
	}
}

void statusDisplay()
{
	printTallyNumber(CAM_LIVE, 0, 0); //Velke cislo pre kameru, ktora je von
	printBigNumber(CAM_READY, 0, 6);  //mensie cislo pre kameru, ktora je von
	printBigNumber(CAMERA, 4, 13);  //mensie cislo pre kameru, ktora je von
}

void printPrepairedMessage()
{
	int ktoraKamera;
	int indexSpravy;
	int poslednyZnak;

	getItem(&myQueue, &ktoraKamera);
	getItem(&myQueue, &indexSpravy);
	getItem(&myQueue, &poslednyZnak);

	if (ktoraKamera & CAMERA_MASK)
	{
		messCamera(indexSpravy);
	}
}

void printRecievedMessage()
{
	int ktoraKamera;
	getItem(&myQueue, &ktoraKamera);

	int pomChar;
	getItem(&myQueue, &pomChar);

	if (ktoraKamera & CAMERA_MASK)
	{
		displayOff();
		clear_display();

		uint8_t posX = 0;
		setXY(posX, 0);

		uint8_t dispalyCharIndex = 0;

		while (pomChar != USART_END_CHAR)
		{
			if (!(dispalyCharIndex < 15))
			{
				posX++;
				setXY(posX, 0);
			}

			getItem(&myQueue, &pomChar);
			sendCharTOMAS(pomChar);
			dispalyCharIndex++;
		}

		displayOn();
	}
	else
	{
		while (pomChar != USART_END_CHAR)
		{
			getItem(&myQueue, &pomChar);
		}
	}
}

void unknowMessenge()
{
	//USART_send(ERROR);

	int pomChar;

	do
	{
		getItem(&myQueue, &pomChar);
		//USART_send(pomChar); //vypisovanie buffra
	} while (pomChar != USART_END_CHAR);

	//USART_send(USART_END_CHAR);
}

int main(void)
{

	setup_Display();

	statusDisplay();

	setup_LED();
	setup_USART();
	
	setup_BUTTON();

	setup_T0_WD();
	setup_T1_showedMess();

	sei(); //Povoleni preruseni

	int temp;

	while (1)
	{
		if (watchdog == false)
		{
			if (new_msg != 0)
			{
				cbi(LED_PORT, B_LED); //vypnutie modej LED, pripad kekz sprava pride pocas svietenia

				getItem(&myQueue, &temp);

				switch (temp) //Prvz znak spravz nesie priznak akz typ spravy je prenasany
				{

				case REFRESH: //Refresh / Change - zmena stavu kamier

				case CHANGED:
					parseCameraStatus();
					refresh_LED(); //Nastavenie LED podla prijatych dat
					if (zobrazenaSprava == false)
					{
						statusDisplay();
					}

					break;

				case MESSAGE_BASIC: //Sprava s informaciou od rezie - predpripravena
					printPrepairedMessage();
					zobrazenaSprava = true;
					timerx_T2 = 0;

					break;

				case MESSAGE_ADVANCE: //Sprava s informaciou od rezie - pisana
					printRecievedMessage();
					zobrazenaSprava = true;
					timerx_T2 = 0;

					break;

				case RESPONSE: //Sprava s informaciou do rezie od kameramanov
					
				default: //sendBackeUnknowMessenge();
					unknowMessenge();
					break;
				}
				new_msg--; //Nulovanie priznaku novej spravy
			}
		}
		else //V pripade ze Watchdog zaznamena chybu spusti sa blikanie modrej LED
		{
			//ToDo - pekny efekt by bol, keby sa zoslabovala
			cbi(LED_PORT, R_LED);
			cbi(LED_PORT, G_LED);

			tbi(LED_PORT, B_LED);
			_delay_ms(WATCHDOG_ERROR);
		}

		if (system_error == true) //V pripade neocakavaneho stavu zariadenia sa vyhlasi chynby stav bez moznosti navratu - nutny restart
		{
			cli(); //Zakazanie preruseni
			while (1)
			{
				tbi(LED_PORT, R_LED); // Blikanie Cervenej Farby
				_delay_ms(FATAL_ERROR);
			}
		}

		_delay_ms(CHECK_MAIN_STATUS); //Cakanie medzi kontrolou stavu systemu
	}
}

/********************obsluhy preruseni****************/

//Ukladanie prichadzajucej spravy do zasobnika
ISR(USART_RX_vect)
{
	watchdog = false; // nulovanie chyboveho stavu dispeja
	timerx_T0 = 0;	// restart casovaca - watchdog

	uint8_t receivedChar = UDR0;
	//USART_send(receivedChar);  //local echo

	if (receivedChar == USART_END_CHAR)
	{
		new_msg++;
	}

	putItem(&myQueue, receivedChar);
}

ISR(TIMER0_COMPA_vect)
{
	timerx_T0++;
	if (timerx_T0 > WATCHDOG_ISR_CNT)
	{
		watchdog = true;
		timerx_T0 = 0;
	}
}

ISR(TIMER2_COMPA_vect)
{
	if (zobrazenaSprava == true)
	{
		timerx_T2++;
		if (timerx_T2 > SHOWMESSENGE_ISR_CNT)
		{
			zobrazenaSprava = false;
			timerx_T2 = 0;
			clear_display();
			statusDisplay();			
		}
	}
}

ISR(PCINT0_vect) 
{
	uint8_t changedbits;
	uint8_t intreading = BUTTON_PIN & 0x7;
	changedbits = intreading ^ portbhistory;
	portbhistory = intreading;
	
	switch(changedbits){

		case 0: //nothing changed
		break;

		case 1: //pcint0 changed
			if(portbhistory & 0x01)
			{
				USART_send(RESPONSE);
				USART_send(CAMERA_MASK);
				USART_send(YES);
				USART_send(USART_END_CHAR);
			}
			
		
		break;

		case 2: //pcint1 changed
		if(portbhistory & 0x02)
		{
			zobrazenaSprava = false;
			timerx_T2 = 0;
			clear_display();
			statusDisplay();
		}
		break;

		case 3: //0+1 changed
	
		break;

		case 4:  //pcint2 changed
		if(portbhistory & 0x04)
		{
		USART_send(RESPONSE);
		USART_send(CAMERA_MASK);
		USART_send(NO);
		USART_send(USART_END_CHAR);
		}
		
		break;

		case 5: //2+0 changed
		USART_send(0x05);
		break;

		case 6: //2+1
		USART_send(0x06);
		break;

		case 7: //all changed

		break;
}

}