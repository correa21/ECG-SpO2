/**
	\file
	\brief
		This source files contains the methods to configure ADC module
		and get the voltage values of the input and displayed in NOKIA LCD
	\author Javier Chavez
	\date	30/10/2019
	\todo
 */

#ifndef RGB_ADC_H_
#define RGB_ADC_H_

#include "ADC.h"
#include "GPIO.h"
#include "stdint.h"
#include "Bits.h"
#include "MK64F12.h"
#include "PIT.h"
#include "NVIC.h"


#define PIT_SAMPLE (0.0125F)
#define CLOCK_RATE (21000000U)

#define MAX_SAMPLES (320U)

typedef enum{IDLE,
			 SAMPLING,
}sampling_status_t;

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This Inits ADC and PIT Instance to get periodic samples of ADC module.

 	 \param[in]
 	 \return void
 	 \todo Implement a mechanism to clear interrupts by a specific pin.
 */
void ECG_ADC_Init(void);

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function sets RGB colors calling FlexTimer functions.

 	 \param[in]  ADC_value ADC averaged sample
 	 \return void
 	 \todo Implement a mechanism to clear interrupts by a specific pin.
 */
void ECG_ADC_set_value(uint16_t ADC_value);

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function gets 10 ADC samples and do the average.

 	 \param[in]  adc_channel ADC Channel to get values of.
 	 \return void
 	 \todo Implement a mechanism to clear interrupts by a specific pin.
 */
void ECG_ADC_get_values(ADC_channel_t adc_channel);

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function triggers when PIT count ends.

 	 \param[in]  void
 	 \return void
 	 \todo Implement a mechanism to clear interrupts by a specific pin.
 */
void ECG_ADC_PIT_Callback(void);

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function stops PIT timer and sets PWM channels to zero.

 	 \param[in]  void.
 	 \return void
 	 \todo Implement a mechanism to clear interrupts by a specific pin.
 */
void ECG_ADC_DeInit(void);

uint8_t ECG_ADC_check_status(void);

uint16_t * ECG_ADC_return_samples(void);



#endif /* RGB_ADC_H_ */
