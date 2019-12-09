/*
 * time.c
 *
 *  Created on: Dec 1, 2019
 *      Author: armando
 */

#include "times.h"


uint32_t milis_g = 0; /* Time elapsed since the program start in ms, it can count till 49 hrs  */


static void ms_count (void)
{
		milis_g++;
};

void time_start (void)
{
	PIT_clock_gating();
	PIT_enable();
	PIT_enable_interrupt(PIT_0);
	PIT_callback_init(PIT_0, ms_count);
	PIT_delay(PIT_0, CLOCK_RATE, MS);
	PIT_enable_timer(PIT_0);
	NVIC_enable_interrupt_and_priotity(PIT_CH0_IRQ, PRIORITY_10);
	NVIC_global_enable_interrupts;
}

uint32_t millis(void)
{
	return (milis_g);
}
