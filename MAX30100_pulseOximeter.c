/*
 * MAX30100_pulseOximeter.c
 *
 *  Created on: Dec 3, 2019
 *      Author: armando
 */

#include "MAX30100_pulseOximeter.h"

PulseOximeter_t pox_g;


static void checkSample()
{
    if ((millis() - pox_g.tsLastSample) > (1.0 / SAMPLING_FREQUENCY * 1000.0)) {
    	pox_g.tsLastSample = millis();
    	MAX30100_update(&(pox_g.sensor.rawRed),&(pox_g.sensor.rawIR));//guardo los valores crudos del sensor
        float irACValue = MAX30100_DCRemoval(pox_g.sensor.rawIR,pox_g.filter.IR_W);//elimino componente de directa del IR
        pox_g.filter.IR_W = getDCW();//guardo el valor W para el filtrado recursivo
        float redACValue = MAX30100_DCRemoval(pox_g.sensor.rawRed,pox_g.filter.Red_W);
        pox_g.filter.Red_W = getDCW();

        // The signal fed to the beat detector is mirrored since the cleanest monotonic spike is below zero

        float semiFilteredPulse = MAX30100_meanDiff_Filter(irACValue, &(pox_g.filter));


        float filteredPulseValue = MAX30100_BWLPFilter(semiFilteredPulse);
        BooleanType beatDetected = MAX30100_beatDet_addSample(filteredPulseValue);

        if (MAX30100_beatDet_getRate() > 0) {
        	pox_g.state = PULSEOXIMETER_STATE_DETECTING;
        	SpO2Calculator_update(irACValue, redACValue, beatDetected);
        }
        else if (pox_g.state == PULSEOXIMETER_STATE_DETECTING)
        {
        	pox_g.state = PULSEOXIMETER_STATE_IDLE;
        	SpO2Calculator_reset();
        }
        if (beatDetected && pox_g.onBeatDetected) {
        	pox_g.onBeatDetected();
        }
    }
};

static void checkCurrentBias()
{
	float decrementarRED = (pox_g.filter.Red_W - pox_g.filter.IR_W);
    // Follower that adjusts the red led current in order to have comparable DC baselines between
    // red and IR leds. The numbers are really magic: the less possible to avoid oscillations
    if (millis() - pox_g.tsLastBiasCheck > CURRENT_ADJUSTMENT_PERIOD_MS) {
        BooleanType changed = FALSE;
        if (((pox_g.filter.IR_W - pox_g.filter.Red_W) > 70000) && (pox_g.redLedPower < MAX30100_LED_CURR_50MA))
        {
            pox_g.redLedPower++;
            changed = TRUE;
        }
        else if (((pox_g.filter.Red_W - pox_g.filter.IR_W) > 70000) && (pox_g.redLedPower > 0))
        {
            pox_g.redLedPower--;
            changed = TRUE;
        }

        if (changed) {
            MAX30100_setLedsCurrent(IR_LED_CURRENT, (LEDCurrent)pox_g.redLedPower);
            pox_g.tsLastCurrentAdjustment = millis();
        }

        pox_g.tsLastBiasCheck = millis();
    }
};

static void checkTemperature()
{
    if ((millis() - pox_g.tsLastTemperaturePoll) > TEMPERATURE_SAMPLING_PERIOD_MS) {
        if (MAX30100_isTemperatureReady()) {
            pox_g.sensor.temp = MAX30100_retrieveTemperature();
        }
        MAX30100_startTemperatureSampling();

        pox_g.tsLastTemperaturePoll = millis();
    }
};



void MAX30100_pulseOximeter_begin(void)
{

	time_start();
    MAX30100_init();
    MAX30100_setMode(MAX30100_MODE_SPO2_HR);
    MAX30100_setLedsCurrent(IR_LED_CURRENT, RED_LED_CURRENT_START);

    /*INITIAL DATA*/
    pox_g.filter.meanFilter.index = 0;
    pox_g.filter.meanFilter.count = 0;
    pox_g.filter.meanFilter.sum = 0;
    pox_g.filter.IR_W = 0;
    pox_g.filter.Red_W = 0;
    pox_g.tsFirstBeatDetected = 0;
    pox_g.redLedPower = (uint8_t)RED_LED_CURRENT_START;
    pox_g.state = PULSEOXIMETER_STATE_IDLE;


   // Start temperature sampling and wait for its completion (blocking)
    MAX30100_startTemperatureSampling();
    while (!MAX30100_isTemperatureReady());
    pox_g.sensor.temp = MAX30100_retrieveTemperature();
};

void MAX30100_pulseOximeter_update(void)
{
    checkSample();
    checkCurrentBias();
    checkTemperature();
};

float MAX30100_pulseOximeter_getHeartRate(void)
{
    return (MAX30100_beatDet_getRate());
};

uint8_t MAX30100_pulseOximeter_getSpO2(void)
{
    return (SpO2Calculator_getSpO2());
};

uint8_t  MAX30100_pulseOximeter_getRedLedCurrentBias(void)
{
    return pox_g.redLedPower;
};

float  MAX30100_pulseOximeter_getTemperature(void)
{
    return pox_g.temperature;
};

void  MAX30100_pulseOximeter_setOnBeatDetectedCallback(void (*cb)())
{
    pox_g.onBeatDetected = cb;
};


