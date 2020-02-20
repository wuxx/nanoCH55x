#include "stdio.h"
#include "CH552.H"

unsigned int i=0;

sbit  KEY = P3^3;
sbit  LED = P3^0;

/* TX:  P3.1/PWM2/TXD */
/* RX:  P3.0/PWM1/RXD */

/* baudrate: 57600 */
main()
{
 
	CfgFsys();	
	mInitSTDIO();
	printf("DEBUG_DATE:"__DATE__"\nDEBUG_TIME:"__TIME__"\n");
	
	while(1)
	{
		while(!KEY);
		LED=!LED;
			i=10000;while(i--);                                                 
	}
}