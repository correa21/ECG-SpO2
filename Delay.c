/*
 * Delay.c
 *
 *  Created on: 17 oct 2019
 *      Author: Javier Chavez
 *
 */
#include "Delay.h"

void delay(uint32_t delay)
{
	volatile uint32_t counter;

	for(counter=delay; counter > 0; counter--) /* Decreasing for loops, because it´s easier for the MCU compare versus zero*/
	{
		__asm("nop");
	}

}

///////////////////////////////////////////////////////////////////////////////
///EOF
///////////////////////////////////////////////////////////////////////////////
