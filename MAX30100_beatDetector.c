/*
 * MAX30100_beatDetector.c
 *
 *  Created on: Nov 30, 2019
 *      Author: armando
 */

#include "MAX30100_beatDetector.h"

BeatDetector_t hrm_g = {BEATDETECTOR_STATE_INIT,
						BEATDETECTOR_MIN_THRESHOLD,
						0,
						0,
						0
						};

BooleanType beat_g = FALSE; /* Flag to indicate when a beat its found */
BooleanType timeSave_g = FALSE;



static void decreaseThreshold (void)
{
	if ((hrm_g.lastMaxValue > 0) && hrm_g.beatPeriod > 0)
	{
		hrm_g.threshold -= hrm_g.lastMaxValue * (1 - BEATDETECTOR_THRESHOLD_FALLOFF_TARGET) / (hrm_g.beatPeriod / BEATDETECTOR_SAMPLES_PERIOD);//a treshold le restamos la division de 0.7/el periodo de beats entre 1/Fs
	}
	else
	{
		// Asymptotic decay
		hrm_g.threshold *= BEATDETECTOR_THRESHOLD_DECAY_FACTOR;
	}

	if (hrm_g.threshold < BEATDETECTOR_MIN_THRESHOLD)
	{
		hrm_g.threshold = BEATDETECTOR_MIN_THRESHOLD;//nunca va a haber un valor menor del threshold que el minimo
	}
}

static BooleanType CheckForbeat (float sample)
{
	BooleanType beatDetected = FALSE;

	switch (hrm_g.state)
	{
		case BEATDETECTOR_STATE_INIT:
			if (millis() > BEATDETECTOR_INIT_HOLDOFF)
			{
				hrm_g.state = BEATDETECTOR_STATE_WAITING;
			}
		break;

		case BEATDETECTOR_STATE_WAITING:
			if (sample > hrm_g.threshold)
			{
				if(sample < BEATDETECTOR_MAX_THRESHOLD)
				{
					hrm_g.threshold = sample;
				}
				else
				{
					hrm_g.threshold = BEATDETECTOR_MAX_THRESHOLD;
				}
				hrm_g.state = BEATDETECTOR_STATE_FOLLOWING_SLOPE;
			}

			// Tracking lost, resetting
			if (millis() - hrm_g.tsLastBeat > BEATDETECTOR_INVALID_READOUT_DELAY)//si el tiempo entre beat y beat es mayor al delay esperado se reinicia la cuenta de todo.
			{
				hrm_g.beatPeriod = 0;
				hrm_g.lastMaxValue = 0;
			}

			decreaseThreshold();//ajustar del threshold
		break;

		case BEATDETECTOR_STATE_FOLLOWING_SLOPE:
			if (sample < hrm_g.threshold)
			{
				hrm_g.state = BEATDETECTOR_STATE_MAYBE_DETECTED;
			}
			else
			{
				if(sample < BEATDETECTOR_MAX_THRESHOLD)
				{
					hrm_g.threshold = sample;
				}
				else
				{
					hrm_g.threshold = BEATDETECTOR_MAX_THRESHOLD;
				}

			}
		break;

		case BEATDETECTOR_STATE_MAYBE_DETECTED:
			if (sample + BEATDETECTOR_STEP_RESILIENCY < hrm_g.threshold)
			{
				// Found a beat
				beatDetected = TRUE;
				hrm_g.lastMaxValue = sample;
				hrm_g.state = BEATDETECTOR_STATE_MASKING;
				float delta = millis() - hrm_g.tsLastBeat;
				if (delta)
				{
					hrm_g.beatPeriod = (BEATDETECTOR_BPFILTER_ALPHA * delta) + ((1 - BEATDETECTOR_BPFILTER_ALPHA) * hrm_g.beatPeriod);
				}

				hrm_g.tsLastBeat = millis();

			}
			else
			{
				hrm_g.state = BEATDETECTOR_STATE_FOLLOWING_SLOPE;
			}
		break;

		case BEATDETECTOR_STATE_MASKING:
			if (millis() - hrm_g.tsLastBeat > BEATDETECTOR_MASKING_HOLDOFF)
			{
				hrm_g.state = BEATDETECTOR_STATE_WAITING;
			}
			decreaseThreshold();
		break;
	}

	return beatDetected;
}


float MAX30100_beatDet_getRate (void)
{
    if (hrm_g.beatPeriod != 0)
    {
        return ((1 / hrm_g.beatPeriod) * 1000 * 60);
    }
    else
    {
        return 0;
    }
};



float MAX30100_beatDet_getCurrentThreshold (void)
{
    return hrm_g.threshold;
}



BooleanType MAX30100_beatDet_addSample(float sample)
{
	return (CheckForbeat(sample));
}
