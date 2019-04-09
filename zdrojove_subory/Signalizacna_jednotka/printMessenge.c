/*
 * printMessenge.c
 *
 * Created: 1/26/2019 2:11:00 PM
 * Author: Tomas Klein
 */

#include "printMessenge.h"
#include "ssd1306.h"

/************************************************************************/
/* Method send prepared message to show on display                      */
/************************************************************************/
void showPreparedMessage(int messageID)
{
	display_turnOff();
	dispaly_clear();

	switch (messageID)
	{
	case 1:
		setXY(1, 5);					//set position
		display_sendString("BLIKA");	//write string on display
		setXY(3, 7);
		display_sendString("TI");
		setXY(5, 5);
		display_sendString("KAMERA");
		break;
		
	case 2:
		setXY(1, 3);
		display_sendString("NEZAOSTRUJE");
		setXY(3, 7);
		display_sendString("TI");
		setXY(5, 5);
		display_sendString("KAMERA");
		break;
	case 3:
		setXY(1, 2);
		display_sendString("PRIBLIZUJE");
		setXY(3, 7);
		display_sendString("TI");
		setXY(5, 5);
		display_sendString("KAMERA");
		break;
		
	default:
		setXY(1, 4);
		display_sendString("TOTO  MA");
		setXY(3, 6);
		display_sendString("ESTE");
		setXY(5, 4);
		display_sendString("NENAUCIL");
		setXY(7, 4);
		display_sendString("ZOBRAZIT");
		break;

	}

	display_turnOn();
}
