/*
 *	Delay functions fro PIC 18F1330 running at 18.432 Mhz
 *	Modified 6-30-17
 *
 */

#include "DELAY16.H"
// #include <pic18.h>


// Millisecond delay routine for 16 Mhz PICs
// Accepts any unsigned integer value
// Will create a delay in milliseconds
// for that value.
void DelayMs(unsigned short i)
{
unsigned short j;
unsigned char k;

	for(j=0; j<i; j++)
	{
		k=163;
		while(k--);
		k=255;
		while(k--);
		k=255;
		while(k--);
		k=255;
		while(k--);
	}
}


// Microsecond delay routine for 16 Mhz PICs
// Accepts any value from 1 to 255 microseconds
/*
void DelayUs(unsigned char i)
{
	while(i--);
}
*/

 
