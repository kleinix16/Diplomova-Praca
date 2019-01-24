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


char RX_buffer[USART_BUFFER];
int RX_index = 0;

int system_error = 0;
int new_msg = 0;
int watchdog = 0;

uint16_t timerx;

void setup_GPIO(void)
{
	DDRC |= (1 << PORTC4) | (1 << PORTC5); //OE, SERDATA
	DDRB |= (1 << PORTB1) | (1 << PORTB2); // SRCLK, RCLK
	sbi(DDRD, PORTD1);					   // TXD USART
	sbi(DDRD, PORTD7);					   // Control Trans. GP2Y
	sbi(PORTD, PORTD2);					   // Pull-up S2
	sbi(DDRD, PORTD5);					   //RES_ESP

	sbi(DDRD, PORTD6); // CO2
	sbi(DDRD, PORTD7); // Control Trans CO2
	//sbi(PORTD,PORTD5);				//HW RESET
	//_delay_ms(10);
	//cbi(PORTD,PORTD5);				// non_RESET
	
	cbi(PORTD,G_LED);
	cbi(PORTD, G_LED);
	cbi(PORTB,B_LED);
}

void setup_T0()
{
	TCCR0A = 0x2;			 //rezim CTC interrupt pri zhode z OCR0A
	TCCR0B = 0x5;			 //clk/1024
	OCR0A = 156;			 //156
	TIMSK0 |= (1 << OCIE0A); //lokalme povolenie prerusenia
}

void setup_USART(void)
{
	DDRD = 0b01001000;
	DDRB = 0b00000010;

	DDRC |= (1 << M0);  // Tx output
	DDRC |= (1 << M1);  // Tx output
	
	DDRD |= (1 << PORTD1);  // Tx output
	DDRD &= ~(1 << PORTD0); //Rx input
	
	
	
	cbi(PORTC, M0);
	cbi(PORTC, M1);
	
	

	UBRR0H = (uint8_t)(UBRR_VALUE >> 8); // BAUDERATE
	UBRR0L = (uint8_t)UBRR_VALUE;
	UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00); //8 bits, parit non, 1 stop
	UCSR0B |= (1 << RXEN0) | (1 << TXEN0);   // enable rx a tx
	UCSR0A |= (1 << U2X0);

	UCSR0B |= (1 << RXCIE0); //RX complete interrupt
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

void refresh_LED()
{
	//Kontrola, ci je kamera LIVE
	if (RX_buffer[1] & CAMERA_MASK)
	{
		cbi(PORTD, R_LED);
	}
	else
	{
		sbi(PORTD, R_LED);
	}

	//Kontrola, ci je kamera READY
	if (RX_buffer[2] & CAMERA_MASK)
	{
		cbi(PORTD, G_LED);
	}
	else
	{
		sbi(PORTD, G_LED);
	}
}



int main(void)
{
	//uint8_t a;
	setup_GPIO();
	setup_USART();
	setup_T0();

	sei();

	while (1)
	{
		if (!watchdog)
		{
			//cbi(PORTB,B_LED);
			
			if (new_msg == 1)
			{
				switch (RX_buffer[0]) //delenie cinnosti podla typu spravy
				{
					//Refresh - zmena stavu kamier
					case 1:
					case 2:
					refresh_LED();
					break;

					//prisla sprava od rezie
					case 'M':
					USART_send('M');
					break;

					//ToDo - pripad, co ak prijata sprava  zacina zle...
					default:
					USART_send(RX_buffer[0]);
				}
				new_msg = 0;
			}
		}
		else
		{
			cbi(PORTD,R_LED); 
			cbi(PORTD,G_LED);
			
			tbi(PORTB, B_LED);
			_delay_ms(150);
		}

		if (system_error)
		{
			cli();
			while (1)
			{
				tbi(PORTD, R_LED);
				_delay_ms(1000);
			}
		}
			
		_delay_ms(100);
	}
}


/********************obsluhy preruseni****************/

//Ukladanie prichadzajucej spravy do zasobnika
ISR(USART_RX_vect)
{

	
	watchdog = 0; // nulovanie chyboveho stavu dispeja
	timerx = 0;   // restart casovaca

	if ((RX_index != USART_BUFFER) && (new_msg == 0))
	{
		RX_buffer[RX_index] = UDR0;
		//USART_send(RX_buffer [RX_index]);  //local echo

		if (RX_buffer[RX_index] == USART_END_CHAR) //
		{
			new_msg = 1;
			RX_index = 0;
			
		}
		else
		{
			RX_index++;
		}
	}
	else
	{
		system_error = 1;
	}
}

ISR(TIMER0_COMPA_vect)
{
	if (timerx > 59999)
	timerx = 0; //600 s
	timerx++;
	if (timerx > 800)
	{
		watchdog = 1;
		timerx = 0;
	}
}