/*
 * MAX30100.h
 *
 *  Created on: Nov 30, 2019
 *      Author: armando
 */

#ifndef MAX30100_H_
#define MAX30100_H_

#include "stdint.h"
#include "I2C.h"
#include "Bits.h"
#include "MAX30100_Registers.h"

#define DEFAULT_MODE                MAX30100_MODE_HRONLY
#define DEFAULT_SAMPLING_RATE       MAX30100_SAMPRATE_100HZ
#define DEFAULT_PULSE_WIDTH         MAX30100_SPC_PW_1600US_16BITS
#define DEFAULT_RED_LED_CURRENT     MAX30100_LED_CURR_50MA
#define DEFAULT_IR_LED_CURRENT      MAX30100_LED_CURR_50MA

#define BYTES_PER_SAMPLE	4

typedef struct MAX30100_s{
	uint16_t rawRed;
	uint16_t rawIR;
	float temp;
}MAX30100_t;

void MAX30100_init(void);

uint8_t MAX30100_rRegister(uint8_t adress);

void MAX30100_setLedsCurrent(LEDCurrent irLedCurrent, LEDCurrent redLedCurrent);

void MAX30100_setMode(Mode mode);

void MAX30100_setLedsPulseWidth(LEDPulseWidth ledPulseWidth);

void MAX30100_setLedsCurrent(LEDCurrent irLedCurrent, LEDCurrent redLedCurrent);

void MAX30100_setSamplingRate(SamplingRate samplingRate);

void MAX30100_setHighresModeEnabled(BooleanType enabled);

void MAX30100_update();

void MAX30100_startTemperatureSampling();

BooleanType MAX30100_isTemperatureReady();

float MAX30100_retrieveTemperature();

#endif /* MAX30100_H_ */
