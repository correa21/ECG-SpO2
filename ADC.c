/*
 * ADC.c
 *
 *  Created on: Oct 28, 2019
 *      Author: Javier Chavez
 */


/** ADC0 SE17  PTE24 Default and Alternative ALT0 */

#include "ADC.h"

void ADC_Init(const ADC_config_t * config_struct)
{
	switch(config_struct->adc_channel)
	{
		case ADC_0:

		    //SIM->SCGC5 |= GPIO_CLOCK_GATING_PORTE;
		   	//PORTC->PCR[bit_24] = PORT_PCR_MUX(0x00); 								//Default configuration

			SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK;

	        ADC0->SC1[0]  &= ~ADC_SC1_ADCH_MASK;                					//Channel 0 from ADC0 is selected as input

	        ADC0->CFG1 |= ADC_CFG1_ADLSMP_MASK;                 					//Long sample is selected to get the desired ADCK cycles
	        ADC0->CFG1 |= (ADC_CFG1_MODE(config_struct->adc_resolution));   		//ADC resolution mode

	        ADC0->CFG2 |= ADC_CFG2_ADLSTS(config_struct->long_sample);      	    //Long sample time select

	        ADC0->SC3 |= ADC_SC3_AVGE_MASK;                    					   //Enable sample average
	        ADC0->SC3 |= ADC_SC3_AVGS(config_struct->adc_average_sample);          //Select hardware average

		break;

		case ADC_1:
			SIM->SCGC3 |= SIM_SCGC3_ADC1_MASK;

	        ADC1->SC1[0]  &= ~ADC_SC1_ADCH_MASK;                //Channel 0 from ADC1 is selected as input

	        ADC1->CFG1 |= ADC_CFG1_ADLSMP_MASK;                 //Long sample is selected to get the desired ADCK cycles
	        ADC1->CFG1 |= (ADC_CFG1_MODE(config_struct->adc_resolution));  //16 bit ADC resolution mode is selected

	        ADC1->CFG2 |= ADC_CFG2_ADLSTS(config_struct->long_sample);      //Long sample time select

	        ADC1->SC3 |= ADC_SC3_ADCO_MASK;
	        ADC1->SC3 |= ADC_SC3_AVGE_MASK;                                         //Enable sample average
	        ADC1->SC3 |= ADC_SC3_AVGS(config_struct->adc_average_sample);          //Select hardware average of 16 samples
		break;
	}
}

uint16_t ADC_getMeasure(ADC_channel_t adc_channel)
{
	volatile uint16_t ADC_value = 0;

		switch(adc_channel)
		{
			case ADC_0:
				ADC0->SC1[0] = ADC_SC1_ADCH(17); //Trigger to start ADC conversion
				while((ADC0->SC1[0] & ADC_SC1_COCO_MASK) == FALSE);
				ADC_value = ADC0->R[0];  //COCO Flag is cleared once the register data is read
				//ADC0->SC1[0] &= ~ADC_SC1_COCO_MASK;
			break;

			case ADC_1:
				while((ADC1->SC1[0] & ADC_SC1_COCO_MASK) == FALSE);
				ADC_value = ADC1->R[0];
			break;
		}

	return (ADC_value);
}
