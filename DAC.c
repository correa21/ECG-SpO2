/*
 * DAC.c
 *
 *  Created on: 4 oct 2019
 *      Author: Javier Chavez
 */

#include "DAC.h"


void DAC_clock_gating(DAC_number_t dac_number)
{
	if(DAC_0 == dac_number)
	{
		SIM->SCGC2 |= DAC0_CLOCK_GATING;
	}
	else
	{
		SIM->SCGC2 |= DAC1_CLOCK_GATING;
	}

}
void DAC_init(DAC_number_t dac_number)
{
	if(DAC_0 == dac_number)
	{
		DAC0->C0 = DAC_C0_DACEN_MASK | DAC_C0_DACRFS_MASK; /** Enables DAC and sets VREF2 */
		DAC0->C1 = ~DAC_C1_DACBFEN_MASK; /** The converted data is always the first word of the buffer*/
		DAC0->C2 = DAC_C2_DACBFRP_MASK; /** Keeps the current value of the buffer read pointer*/
	}
	else
	{
		DAC1->C0 = DAC_C0_DACEN_MASK | DAC_C0_DACRFS_MASK; /** Enables DAC and sets VREF2 */
		DAC1->C1 = ~DAC_C1_DACBFEN_MASK; /** The converted data is always the first word of the buffer*/
		DAC1->C2 = DAC_C2_DACBFRP_MASK; /** Keeps the current value of the buffer read pointer*/
	}
}

void DAC_set_value(DAC_number_t dac_number, uint8_t index, uint16_t dac_value)
{
	if(DAC_0 == dac_number)
	{
		DAC0->DAT[index].DATL = (uint8_t)(0xFFU & dac_value);         /** Low 8-bit. */
		DAC0->DAT[index].DATH = (uint8_t)((0xF00U & dac_value) >> 8); /** High 4-bit. */
	}
	else
	{
		DAC1->DAT[index].DATL = (uint8_t)(0xFFU & dac_value);         /** Low 8-bit. */
		DAC1->DAT[index].DATH = (uint8_t)((0xF00U & dac_value) >> 8); /** High 4-bit. */
	}

}

void DAC_deinit(DAC_number_t dac_number)
{
	if(DAC_0 == dac_number)
	{
		DAC0->C0 = ~DAC_C0_DACEN_MASK;
	}
	else
	{
		DAC1->C0 = ~DAC_C0_DACEN_MASK;
	}
}

void DAC_set_buff_pointer(DAC_number_t dac_number, uint8_t index)
{
	if(DAC_0 == dac_number)
	{
		DAC0->C2 |= DAC_C2_DACBFRP(index);
	}
	else
	{
		DAC1->C2 |= DAC_C2_DACBFRP(index);
	}
}



