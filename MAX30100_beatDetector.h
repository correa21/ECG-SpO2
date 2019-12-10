/*
 * MAX30100_beatDetector.h
 *
 *  Created on: Nov 30, 2019
 *      Author: armando
 */

#ifndef MAX30100_BEATDETECTOR_H_
#define MAX30100_BEATDETECTOR_H_

#include "stdint.h"
#include "Bits.h"
#include "times.h"

#define BEATDETECTOR_INIT_HOLDOFF                2000    // in ms, how long to wait before counting
#define BEATDETECTOR_MASKING_HOLDOFF             200     // in ms, non-retriggerable window after beat detection
#define BEATDETECTOR_BPFILTER_ALPHA             (0.6F)     // EMA factor for the beat period value
#define BEATDETECTOR_MIN_THRESHOLD               20      // minimum threshold (filtered) value
#define BEATDETECTOR_MAX_THRESHOLD               800     // maximum threshold (filtered) value
#define BEATDETECTOR_STEP_RESILIENCY             30      // maximum negative jump that triggers the beat edge
#define BEATDETECTOR_THRESHOLD_FALLOFF_TARGET   (0.3F)     // thr chasing factor of the max value when beat
#define BEATDETECTOR_THRESHOLD_DECAY_FACTOR     (0.99F)    // thr chasing factor when no beat
#define BEATDETECTOR_INVALID_READOUT_DELAY       2000    // in ms, no-beat time to cause a reset
#define BEATDETECTOR_SAMPLES_PERIOD              10      // in ms, 1/Fs


typedef enum BeatDetectorState {
    BEATDETECTOR_STATE_INIT,
    BEATDETECTOR_STATE_WAITING,
    BEATDETECTOR_STATE_FOLLOWING_SLOPE,
    BEATDETECTOR_STATE_MAYBE_DETECTED,
    BEATDETECTOR_STATE_MASKING
} BeatDetectorState_t;

typedef struct BeatDetector_s
{
	BeatDetectorState_t state;//current state
	float threshold;//value of current threshold
	float beatPeriod;
	float lastMaxValue;
	uint32_t tsLastBeat;
}BeatDetector_t;

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function returns the beats per minute calculated with the beatPeriod.

 	 \param[in] void
 	 \return float
 	 \todo modify the calculation method.
 */
float MAX30100_beatDet_getRate (void);

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function returns the current threshold for debbuggin purposes

 	 \param[in] void
 	 \return float
 */
float MAX30100_beatDet_getCurrentThreshold (void);

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function check if the sample it's or not a beat and return TRUE or FALSE.

 	 \param[in] float sample
 	 \return BooleanType
 */
BooleanType MAX30100_beatDet_addSample(float sample);

#endif /* MAX30100_BEATDETECTOR_H_ */
