/*
 * PIT.c
 *
 *  Created on: 17 sep 2019
 *      Author: Javier Chavez
 */
#include "PIT.h"

static pit_interrupt_flags_t g_pit_status_flag = {0};

static void (*PIT_0_callback)(void) = 0;
static void (*PIT_1_callback)(void) = 0;
static void (*PIT_2_callback)(void) = 0;
static void (*PIT_3_callback)(void) = 0;



void PIT_delay(PIT_timer_t pit_timer, My_float_pit_t system_clock,
			   My_float_pit_t delay)
{
	uint32_t LDVAL = 0;
	system_clock /= 2;
	My_float_pit_t clock_period = (1 / system_clock);
	LDVAL = (uint32_t) (delay / clock_period);
	LDVAL -= 1;


	PIT->CHANNEL[pit_timer].LDVAL = LDVAL;
}

void PIT_clock_gating(void)
{
	SIM->SCGC6 |= SIM_SCGC6_PIT_MASK;
}

uint8_t PIT_get_interrupt_flag_status(PIT_timer_t pit)
{
	uint8_t status = 0;

	switch (pit)/** Selecting PIT interrupt flag*/
	{
		case PIT_0:
			status = g_pit_status_flag.flag_pit_0;
		break;

		case PIT_1:
			status = g_pit_status_flag.flag_pit_1;
		break;

		case PIT_2:
			status = g_pit_status_flag.flag_pit_2;
		break;

		case PIT_3:
			status = g_pit_status_flag.flag_pit_3;
		break;

		default: /**If doesn't exist the option*/
		break;
	}
	return(status);
}


void PIT_clear_interrupt_flag(PIT_timer_t pit)
{

	switch (pit)/** Selecting PIT interrupt flag*/
	{
		case PIT_0:
			g_pit_status_flag.flag_pit_0 = FALSE;
		break;

		case PIT_1:
			g_pit_status_flag.flag_pit_1 = FALSE;
		break;

		case PIT_2:
			g_pit_status_flag.flag_pit_2 = FALSE;
		break;

		case PIT_3:
			g_pit_status_flag.flag_pit_3 = FALSE;
		break;

		default: /**If doesn't exist the option*/
			g_pit_status_flag.flag_pit_0 = FALSE;
			g_pit_status_flag.flag_pit_1 = FALSE;
			g_pit_status_flag.flag_pit_2 = FALSE;
			g_pit_status_flag.flag_pit_3 = FALSE;
		break;
	}

}


void PIT_enable(void)
{
	PIT->MCR &= ~PIT_MCR_MDIS_MASK;
	PIT->MCR |=  PIT_MCR_FRZ_MASK;
}

void PIT_enable_interrupt(PIT_timer_t pit)
{
	PIT->CHANNEL[pit].TCTRL |= PIT_TCTRL_TIE_MASK;
}

void PIT_enable_timer(PIT_timer_t pit)
{
	PIT->CHANNEL[pit].TCTRL |= PIT_TCTRL_TEN_MASK;
}

void PIT_callback_init(PIT_timer_t pit, void (*handler)(void))
{

	switch(pit)
	{
		case PIT_0:
			PIT_0_callback = handler;
		break;

		case PIT_1:
			PIT_1_callback = handler;
		break;

		case PIT_2:
			PIT_2_callback = handler;
		break;

		case PIT_3:
			PIT_3_callback = handler;
		break;

		default:
		break;

	}
}

void PIT_stop_timer(PIT_timer_t pit)
{
	PIT->CHANNEL[pit].TCTRL &= ~PIT_TCTRL_TIE_MASK;   /** Disables pit instance timer interrupts*/
	PIT->CHANNEL[pit].TCTRL &= ~PIT_TCTRL_TEN_MASK;	  /** Disables timer count */

}

/********************************************************************************************/
/*!
		PIT Interrupt Service Routines
 */
void PIT0_IRQHandler(void)
{

	PIT->CHANNEL[0].TFLG |= PIT_TFLG_TIF_MASK;
	uint32_t dummyRead = PIT->CHANNEL[0].TCTRL; //read control register for clear PIT flag, this is silicon bug
	if(PIT_0_callback)
	{
		PIT_0_callback();
	}
	g_pit_status_flag.flag_pit_0 = TRUE;

}

void PIT1_IRQHandler(void)
{
	PIT->CHANNEL[1].TFLG |= PIT_TFLG_TIF_MASK;
	uint32_t dummyRead = PIT->CHANNEL[1].TCTRL; //read control register for clear PIT flag, this is silicon bug
	g_pit_status_flag.flag_pit_1 = TRUE;
	if(PIT_1_callback)
	{
		PIT_1_callback();
	}
}

void PIT2_IRQHandler(void)
{
	PIT->CHANNEL[2].TFLG |= PIT_TFLG_TIF_MASK;
	uint32_t dummyRead = PIT->CHANNEL[2].TCTRL; //read control register for clear PIT flag, this is silicon bug
	g_pit_status_flag.flag_pit_2 = TRUE;
	if(PIT_2_callback)
	{
		PIT_2_callback();
	}
}

void PIT3_IRQHandler(void)
{
	PIT->CHANNEL[3].TFLG |= PIT_TFLG_TIF_MASK;
	uint32_t dummyRead = PIT->CHANNEL[3].TCTRL; //read control register for clear PIT flag, this is silicon bug
	g_pit_status_flag.flag_pit_3 = TRUE;
	if(PIT_3_callback)
	{
		PIT_3_callback();
	}
}

///////////////////////////////////////////////////////////////////////////////
///EOF
///////////////////////////////////////////////////////////////////////////////
