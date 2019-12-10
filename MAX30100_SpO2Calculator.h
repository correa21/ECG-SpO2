/*
 * MAX30100_SpO2Calculator.h
 *
 *  Created on: Dec 3, 2019
 *      Author: armando
 */

#ifndef MAX30100_SPO2CALCULATOR_H_
#define MAX30100_SPO2CALCULATOR_H_

#include <stdint.h>
#include <math.h>
#include "Bits.h"

#define CALCULATE_EVERY_N_BEATS         3

typedef struct SpO2Calculator_s
{
	float irACValueSqSum;
	float redACValueSqSum;
	uint8_t beatsDetectedNum;
	uint32_t samplesRecorded;
	float spO2;
}SpO2Calculator_t;

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function receive the ac values from the red led reading and IR led reading.
 	 	 	 it calculate the SpO2 is beatDetected is TRUE

 	 \param[in] float irACValue, float redACValue, BooleanType beatDetected
 	 \return void
 */
void SpO2Calculator_update(float irACValue, float redACValue, BooleanType beatDetected);

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 Reset the values from a internal SpO2Calculator_t variable.

 	 \param[in] void
 	 \return void
 */
void SpO2Calculator_reset(void);

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 Return the SpO2 value.

 	 \param[in] void
 	 \return float
 */
float SpO2Calculator_getSpO2(void);


#endif /* MAX30100_SPO2CALCULATOR_H_ */
