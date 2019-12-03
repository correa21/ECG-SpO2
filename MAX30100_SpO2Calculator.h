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
	uint8_t spO2;
}SpO2Calculator_t;

void SpO2Calculator_update(float irACValue, float redACValue, BooleanType beatDetected);
void SpO2Calculator_reset(void);
uint8_t SpO2Calculator_getSpO2(void);


#endif /* MAX30100_SPO2CALCULATOR_H_ */
