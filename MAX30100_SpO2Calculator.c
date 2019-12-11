/*
 * MAX30100_SpO2Calculator.c
 *
 *  Created on: Dec 3, 2019
 *      Author: armando
 */


#include "MAX30100_SpO2Calculator.h"

SpO2Calculator_t SpO2 ={0};

static const uint8_t spO2LUT[43] = {100,100,100,100,99,99,99,99,99,99,98,98,98,98,
							  98,97,97,97,97,97,97,96,96,96,96,96,96,95,95,
							  95,95,95,95,94,94,94,94,94,93,93,93,93,93};



#define N 10


void SpO2Calculator_update(float irACValue, float redACValue, BooleanType beatDetected)
{
	SpO2.irACValueSqSum += irACValue * irACValue;
	SpO2.redACValueSqSum += redACValue * redACValue;
	SpO2.samplesRecorded++;

	    if (beatDetected)
	    {
	    	SpO2.beatsDetectedNum++;
	        if (SpO2.beatsDetectedNum == CALCULATE_EVERY_N_BEATS) {
	        	float acSqRatio = 100.0 * log((SpO2.redACValueSqSum  / SpO2.samplesRecorded)) / log((SpO2.irACValueSqSum / SpO2.samplesRecorded));

	            SpO2Calculator_reset();

	            SpO2.spO2 = acSqRatio; //spO2LUT[index];
	        }
	    }
};

void SpO2Calculator_reset(void)
{
	SpO2.samplesRecorded = 0;
	SpO2.redACValueSqSum = 0;
	SpO2.irACValueSqSum = 0;
	SpO2.beatsDetectedNum = 0;
	SpO2.spO2 = 0;
};


float SpO2Calculator_getSpO2(void)
{
    return SpO2.spO2;
}
