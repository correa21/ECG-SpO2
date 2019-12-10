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
#include "Delay.h"
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

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function init the sensor in HIG RESOLUTION MODE.

 	 \param[in] void
 	 \return void
 */
void MAX30100_init(void);

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function reds a register from the MAX30100.

 	 \param[in] uint8_t adress
 	 \return uint8_t
 */
uint8_t MAX30100_rRegister(uint8_t adress);

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function sets the LED's current.

 	 \param[in] LEDCurrent irLedCurrent, LEDCurrent redLedCurrent
 	 \return void
 */
void MAX30100_setLedsCurrent(LEDCurrent irLedCurrent, LEDCurrent redLedCurrent);

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function sets the MAX30100 mode.

 	 \param[in] Mode mode
 	 \return void
 */
void MAX30100_setMode(Mode mode);

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function sets the MAX30100 LED pulse width.

 	 \param[in] LEDPulseWidth ledPulseWidth
 	 \return void
 */
void MAX30100_setLedsPulseWidth(LEDPulseWidth ledPulseWidth);

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function sets the MAX30100 LED pulse width.

 	 \param[in] LEDPulseWidth ledPulseWidth
 	 \return void
 */
void MAX30100_setLedsCurrent(LEDCurrent irLedCurrent, LEDCurrent redLedCurrent);

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function sets the MAX30100 LED pulse width.

 	 \param[in] LEDPulseWidth ledPulseWidth
 	 \return void
 */
void MAX30100_setSamplingRate(SamplingRate samplingRate);

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function sets the MAX30100 LED pulse width.

 	 \param[in] LEDPulseWidth ledPulseWidth
 	 \return void
 */
void MAX30100_setHighresModeEnabled(BooleanType enabled);

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function updates the raw values of IR and Red readings.

 	 \param[in] uint16_t*red, uint16_t* ir
 	 \return void
 */
void MAX30100_update(uint16_t*red, uint16_t* ir);

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function start the temperature sampling.

 	 \param[in] LEDPulseWidth ledPulseWidth
 	 \return void
 */
void MAX30100_startTemperatureSampling(void);

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function tells if the temperature measure is ready or not.

 	 \param[in] void
 	 \return BooleanType
 */
BooleanType MAX30100_isTemperatureReady(void);

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function returns the temperature of the chip.

 	 \param[in] void
 	 \return float
 */
float MAX30100_retrieveTemperature(void);

#endif /* MAX30100_H_ */
