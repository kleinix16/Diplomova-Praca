/*
 * I2C.c
 *
 * Created: 22-12-2017 19:00:37
 * Author: osc
 * Source: https://github.com/elo1957/OLED-ssd1306-atmega
 */ 

#ifndef I2C_H_
#define I2C_H_
#define SCL_CLK 100000

char write_address;

#include <util/delay.h>

#define BITRATE(TWSR)	((F_CPU/SCL_CLK)-16)/(2*pow(4,(TWSR&((1<<TWPS0)|(1<<TWPS1)))))
char read_addres;

void I2C_Init()	;
uint8_t I2C_Start(char write_address);/* I2C start function */
uint8_t I2C_Repeated_Start(char read_address); /* I2C repeated start function */
uint8_t I2C_Write(char data);	/* I2C write function */
char I2C_Read_Ack()	;	/* I2C read ack function */
char I2C_Read_Nack();		/* I2C read nack function */
void I2C_Stop()	;		/* I2C stop function */
#endif /* I2C_H_ */