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

	switch (typeOfError)
	{
	case 1:
		setXY(1, 5);
		sendStr("BLIKA");
		setXY(3, 7);
		sendStr("TI");
		setXY(5, 5);
		sendStr("KAMERA");
		break;
		
	case 2:
		setXY(1, 3);
		sendStr("NEZAOSTRUJE");
		setXY(3, 7);
		sendStr("TI");
		setXY(5, 5);
		sendStr("KAMERA");
		break;
	case 3:
		setXY(1, 2);
		sendStr("PRIBLIZUJE");
		setXY(3, 7);
		sendStr("TI");
		setXY(5, 5);
		sendStr("KAMERA");
		break;
		
	default:
		setXY(1, 4);
		sendStr("TOTO  MA");
		setXY(3, 6);
		sendStr("ESTE");
		setXY(5, 4);
		sendStr("NENAUCIL");
		setXY(7, 4);
		sendStr("ZOBRAZIT");
		break;

	}

	displayOn();
}
