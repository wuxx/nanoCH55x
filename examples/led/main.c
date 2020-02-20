#include "CH552.H"

unsigned int i=0;

//sbit  KEY = P3^3;
sbit  LED1 = P3^2;
//sbit  LED2 = P3^4;
main()
{
    
    while(1)
    {
			//while(!KEY);
			LED1=!LED1;
			//LED2=!LED2;
        i=10000;while(i--);                                                 
    }
}