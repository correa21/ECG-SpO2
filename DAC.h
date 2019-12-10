/*
 * DAC.h
 *
 *  Created on: 4 oct 2019
 *      Author: Javier Chavez
 */

#ifndef DAC_H_
#define DAC_H_


#include "MK64F12.h"
#include "GPIO.h"
#include "Bits.h"

#include "PIT.h"

#include "NVIC.h"


#define DAC0_CLOCK_GATING	0x001000
#define DAC1_CLOCK_GATING	0x002000

#define LED_GEN_1 bit_10
#define LED_GEN_2 bit_11



typedef enum{DAC_0,
			 DAC_1
}DAC_number_t;

#define DAC_LOW_INDEX   0U

/********************************************************************************************/
/*!
	 \brief	This function enables the DAC clocks
	 \param[in]  DAC_number_t DAC instance
	 \return void
 */
void DAC_clock_gating(DAC_number_t);

/********************************************************************************************/
/*!
	 \brief	 This function sets all the configuration parameters in the instance selected
	 \param[in]  DAC_number_t DAC instance
	 \return void
 */
void DAC_init(DAC_number_t);

/********************************************************************************************/
/*!
	 \brief	 This function sets the output value in the dac buffer registers
	 \param[in]  DAC_number_t DAC instance
	 \param[in]  uint8_t DAC index to be loaded
	 \param[in]  uint16_t value to be assigned
	 \return void
 */
void DAC_set_value(DAC_number_t, uint8_t, uint16_t);

/********************************************************************************************/
/*!
	 \brief	 This function stops the DAC
	 \param[in]  DAC_number_t DAC instance
	 \return void
 */
void DAC_deinit(DAC_number_t);

/********************************************************************************************/
/*!
	 \brief	 This function sets the index to a specific address
	 \param[in]  DAC_number_t DAC instance
	 \param[in]  uint8_t DAC index to be loaded
	 \return void
 */
void DAC_set_buff_pointer(DAC_number_t, uint8_t);

#endif /* DAC_H_ */
