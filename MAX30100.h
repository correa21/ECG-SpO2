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

#define BYTES_PER_SAMPLE	4

uint8_t MAX30100_rRegister(uint8_t adress);

void MAX30100_setLedsCurrent(LEDCurrent irLedCurrent, LEDCurrent redLedCurrent);

void MAX30100_setMode(Mode mode);

void MAX30100_setLedsPulseWidth(LEDPulseWidth ledPulseWidth);

void MAX30100_setLedsCurrent(LEDCurrent irLedCurrent, LEDCurrent redLedCurrent);

void MAX30100_setSamplingRate(SamplingRate samplingRate);

void MAX30100_setHighresModeEnabled(BooleanType enabled);

void MAX30100_update();

void MAX30100_startTemperatureSampling();

void MAX30100_isTemperatureReady();

float MAX30100_retrieveTemperature();

#endif /* MAX30100_H_ */
