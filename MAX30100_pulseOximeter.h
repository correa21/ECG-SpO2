/*
 * MAX30100_pulseOximeter.h
 *
 *  Created on: Dec 3, 2019
 *      Author: armando
 */

#ifndef MAX30100_PULSEOXIMETER_H_
#define MAX30100_PULSEOXIMETER_H_


#define SAMPLING_FREQUENCY                  100
#define CURRENT_ADJUSTMENT_PERIOD_MS        500
#define IR_LED_CURRENT                      MAX30100_LED_CURR_50MA
#define RED_LED_CURRENT_START               MAX30100_LED_CURR_11MA
#define DC_REMOVER_ALPHA                    0.95
#define TEMPERATURE_SAMPLING_PERIOD_MS      2000

#include <stdint.h>

#include "MAX30100.h"
#include "MAX30100_BeatDetector.h"
#include "MAX30100_filters.h"
#include "MAX30100_SpO2Calculator.h"
#include "times.h"

typedef enum PulseOximeterState {
    PULSEOXIMETER_STATE_INIT,
    PULSEOXIMETER_STATE_IDLE,
    PULSEOXIMETER_STATE_DETECTING
} PulseOximeterState;


typedef struct PulseOximeter_s
{
	MAX30100_t sensor;					/*datos crudos del sensor*/
	MAX30100_FILTER_t filter;			/*resultados de filtros usados para otros desrollos y datos crudos*/
	PulseOximeterState state;			/*estado del oximetro*/
	uint32_t tsFirstBeatDetected;
	uint32_t tsLastBeatDetected;
	uint32_t tsLastSample;
	uint32_t tsLastBiasCheck;
	uint32_t tsLastCurrentAdjustment;
	uint32_t tsLastTemperaturePoll;
	uint8_t redLedPower;
	float temperature;
	void (*onBeatDetected)();	/*Callback function to do when beat is detected*/
}PulseOximeter_t;

void MAX30100_pulseOximeter_begin(void);

void MAX30100_pulseOximeter_update(void);

uint8_t MAX30100_pulseOximeter_getSpO2(void);

uint8_t  MAX30100_pulseOximeter_getRedLedCurrentBias(void);

float  MAX30100_pulseOximeter_getTemperature(void);

float MAX30100_pulseOximeter_getHeartRate(void);

void MAX30100_pulseOximeter_setOnBeatDetectedCallback(void (*cb)());

#endif /* MAX30100_PULSEOXIMETER_H_ */
