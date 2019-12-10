/*
 * time.h
 *
 *  Created on: Dec 1, 2019
 *      Author: armando
 */

#ifndef TIMES_H_
#define TIMES_H_

#include "PIT.h"
#include "NVIC.h"

#define MS										(.001F)  // delay for PIT
#define CLOCK_RATE								(21000000U)//system clk
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This functions start a PIT with a millisecond delay
 	 	 	 and increment a counter every interruption.

 	 \param[in] void
 	 \return void
 */
void time_start (void);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function returns milli seconds passed since the time_star function were executed.

 	 \param[in] void
 	 \return uint32_t
 */
uint32_t millis (void);

#endif /* TIMES_H_ */
