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

#include <stdbool.h>


char RX_buffer[USART_BUFFER];
int RX_index = 0;

bool system_error = false;
bool new_msg = false;
bool watchdog = false;

uint16_t timerx;

//uint8_t _i2c_address = 0X78; // this works 0X3C or 0X3D does not
uint8_t display_buffer[1024];

///////////////////////////////////////////////////////////
// Transfers the local buffer to the CGRAM in the SSD1306
void transferDisplayBuffer()
{
	uint8_t j = 0;

	// set the Column and Page addresses to 0,0
	setColAddress();
	setPageAddress();

	I2C_Start(_i2c_address);
	//I2C_Write(_i2c_address);
	I2C_Write(0X40); // data not command
	for (j = 0; j < 1024; j++)
	{
		I2C_Write(display_buffer[j]);
	}

	I2C_Stop();
}

void initDisplayBuffer(){
	memset(display_buffer, 0X04, 1024); // tried other values
}

void test()
{
	// Initialze SSD1306 OLED display
	reset_display(); // Clear screen
	setXY(0, 0);	 // Set cursor position, start of line 0
	sendStr(" Klein Tomas");
	setXY(1, 1); // Set cursor position, start of line 1
	sendStr("Mlein Tomas");
	setXY(2, 0); // Set cursor position, start of line 2
	sendStr("Klein Tomas");
	setXY(2, 10); // Set cursor position, line 2 10th character
	sendStr("CA");
	setXY(3, 10); // Set cursor position, line 2 10th character
	sendStr("TEST");
}

void setup_LED(void)
{
	sbi(LED_DDR, G_LED);	//Nastavenie pinu ako vystup
	sbi(LED_DDR, R_LED);
	//sbi(LED_DDR, B_LED);	
	
	cbi(LED_PORT, G_LED);	//Nastavenie pociatocnej hodnoty - LEDky nestivtia
	cbi(LED_PORT, R_LED);	
	//cbi(LED_PORT, B_LED);	
	
	sbi(DDRB, B_LED);   //Modra an testovacej doske
	cbi(PORTB, B_LED);
}

void setup_BUTTON(void)
{
	sbi(BUTTON_PORT, BT_1); //Nastavovanie Pull-Up rezistorov
	sbi(BUTTON_PORT, BT_2); 
	sbi(BUTTON_PORT, BT_3); 
	
	cbi(BUTTON_DDR, BT_1);	//Nastavenie pinu ako vstup
	cbi(BUTTON_DDR, BT_2);	
	cbi(BUTTON_DDR, BT_3);	
}

void setup_T0_WD()
{
	TCCR0A = 0x2;			 //rezim CTC interrupt pri zhode z OCR0A
	TCCR0B = 0x5;			 //clk/1024
	OCR0A = WATCHDOG_ISR_CMP;	 
	TIMSK0 |= (1 << OCIE0A); //lokalme povolenie prerusenia
}

void setup_USART(void)
{
	//Nastavovanie AUX portu ako vstup 
	
	//Nastavovanie M0 a M1 ako vystup s urovnou 0
	DDRC  |= (1 << RFM_M0) | (1 << RFM_M1);
	PORTC &= ~(1 << RFM_M0) & ~(1 << RFM_M1);
	
	//Nastavovanie RX a TX 
	DDRD |=  (1 << USART_TX);  // Tx output
	
	DDRD &= ~(1 << USART_RX);  // Rx input
	PORTD |=  (1 << USART_RX); // RX - Pull Up rezistor
	
	
	UBRR0H = (uint8_t)(UBRR_VALUE >> 8); // BAUDERATE
	UBRR0L = (uint8_t)UBRR_VALUE;
	
	UCSR0B|=(1<<TXEN0); //enable TX
	UCSR0B|=(1<<RXEN0); //enable RX
	UCSR0B|=(1<<RXCIE0); //RX complete interrupt
	UCSR0C|=(1<<UCSZ01)|(1<<UCSZ00); // no parity, 1 stop bit, 8-bit data
	UCSR0A|= (1 << U2X0);//USART Double Transmission Speed

}

//ToDo - vytvor funkciu na poslanie stringu

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

void refresh_LED()
{
	//Kontrola, ci je kamera LIVE
	if (RX_buffer[1] & CAMERA_MASK)			//Aplikovanie masky pre zistenie stavu kamerz
	{
		cbi(LED_PORT, R_LED);					//Rozsvietenie cervenej LED
	}
	else
	{
		sbi(LED_PORT, R_LED);					//Zhsnutie cervenej LED
		
		//Kontrola, ci je kamera READY
		if (RX_buffer[2] & CAMERA_MASK)
		{
			cbi(LED_PORT, G_LED);				//Rozsvietenie zelenej
		}
		else
		{
			sbi(LED_PORT, G_LED);				//Zhasnutie zelenej
		}
	}

	
}

int main(void)
{
	
	I2C_Init();
	
	_delay_ms(10);
	InitializeDisplay();
	
	clear_display();
	
	
	//setup()
	
	int j = 0;
	while(1)
	{
		for (uint8_t i = 0; i<8; i++)
		{
			//clear_display();
			printTallyNumber(i,0,0);
			_delay_ms(100);
		}
		printBigNumber(j, 0, 13);
		j++;
		
	}
}

int main2(void)
{
	setup_LED();
	setup_USART();
	setup_T0_WD();

	sei();			//Povoleni preruseni

	while (1)
	{
		if (watchdog == false)
		{
			cbi(PORTB,B_LED);					//vypnutie modej LED, pripad kekz sprava pride pocas svietenia 
			
			if (new_msg == true)
			{
				switch (RX_buffer[0])			//Prvz znak spravz nesie priznak akz typ spravy je prenasany
				{
					//Refresh / Change - zmena stavu kamier
					case REFRESH:
					
					case CHANGED:
							refresh_LED();		//Nastavenie LED podla prijatych dat
							break;

					//Sprava s informaciou od rezie
					case MESSAGE:
							USART_send('M');
							break;
							
					//Sprava s informaciou od rezie
					case RESPONSE:
							USART_send('R');
							break;

						//ToDo - pripad, co ak prijata sprava  zacina zle...
					default:
							USART_send(RX_buffer[0]);
							break;
				}
				new_msg = false;					//Nulovanie priznaku novej spravy
			}
		}
		else									//V pripade ze Watchdog zaznamena chybu spusti sa blikanie modrej LED
		{
			//ToDo - pekny efekt by bol, keby sa zoslabovala
			cbi(LED_PORT,R_LED); 
			cbi(LED_PORT,G_LED);
			
			tbi(PORTB, B_LED);
			_delay_ms(WATCHDOG_ERROR);
		}

		if (system_error == true)		//V pripade neocakavaneho stavu zariadenia sa vyhlasi chynby stav bez moznosti navratu - nutny restart
		{
			cli();						//Zakazanie preruseni
			while (1)
			{
				tbi(LED_PORT, R_LED);		// Blikanie Cervenej Farby
				_delay_ms(FATAL_ERROR);
			}
		}
			
		_delay_ms(CHECK_MAIN_STATUS);	//Cakanie medzi kontrolou stavu systemu	
	}
}


/********************obsluhy preruseni****************/

//Ukladanie prichadzajucej spravy do zasobnika
ISR(USART_RX_vect)
{
	watchdog = false;	// nulovanie chyboveho stavu dispeja
	timerx = 0;			// restart casovaca - watchdog

	if (new_msg == false)
	{
		RX_buffer[RX_index] = UDR0;
		//USART_send(RX_buffer [RX_index]);  //local echo

		if (RX_buffer[RX_index] == USART_END_CHAR) //Kontrola na koncovi znak spravy
		{
			new_msg = true;
			RX_index = 0;
		}
		else
		{
			RX_index++; 
		
			if (RX_index >= USART_BUFFER)   // kontrola na pretecenie zasobnika
			{ 
				system_error = true;
			}
		}
	}
}

ISR(TIMER0_COMPA_vect)
{
	timerx++;
	if (timerx > WATCHDOG_ISR_CNT)
	{
		watchdog = true;
		timerx = 0;
	}
}