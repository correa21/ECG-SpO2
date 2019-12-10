
/*
 * ADC.h
 *
 *  Created on: Oct 28, 2019
 *      Author: Javier Chavez
 *
 */

#ifndef ADC_H_
#define ADC_H_

#include "GPIO.h"
#include "stdint.h"
#include "Bits.h"
#include "MK64F12.h"


typedef enum{ADC_0,
		     ADC_1,
} ADC_channel_t;

typedef enum{ADC_8BIT,
	         ADC_12BIT,
			 ADC_10BIT,
			 ADC_16BIT,
}ADC_Resolution_Mode_t;

typedef enum{BUS_CLK,
	         ALT_CLK2,
			 ALT_CLK,
			 ADACK,
}ADC_bus_clock_t;

typedef enum{SAMPLE_AVG_4,
	         SAMPLE_AVG_8,
			 SAMPLE_AVG_16,
			 SAMPLE_AVG_32,
}ADC_average_sample_t;

typedef enum{LONG_SAMPLE_20,
		     LONG_SMAPLE_12,
			 LONG_SAMPLE_6,
	         LONG_SAMPLE_2,
}ADC_Long_sample_t;

typedef struct
{
	ADC_channel_t adc_channel;
	ADC_Resolution_Mode_t adc_resolution;
	ADC_bus_clock_t adc_clock;
	ADC_average_sample_t adc_average_sample;
	ADC_Long_sample_t long_sample;

}ADC_config_t;

void ADC_Init(const ADC_config_t * config_struct);

uint16_t ADC_getMeasure(ADC_channel_t adc_channel);


#endif /* ADC_H_ */

