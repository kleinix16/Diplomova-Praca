/*
 * printMessenge.c
 *
 * Created: 1/26/2019 2:11:00 PM
 *  Author: klein
 */ 

#include "printMessenge.h"
#include "ssd1306.h"

void messCamera(int typeOfError)
{
	displayOff();
	clear_display();
	
	switch(typeOfError){
		case 0:
				setXY(1, 5);
				sendStr("BLIKA");
				break;
		case 1:
				setXY(1, 3);
				sendStr("NEZAOSTRUJE");
				break;
		case 3:
				setXY(1, 2);
				sendStr("PRIBLIZUJE");
				break;
		default:
				setXY(1, 4);
				sendStr("NEFUNGUJE");
				break;
	}
				
	setXY(3, 7);
	sendStr("TI");
	setXY(5, 5);
	sendStr("KAMERA");
	
	displayOn();
}
