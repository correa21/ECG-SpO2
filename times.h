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

void time_start (void);
uint32_t millis (void);

#endif /* TIMES_H_ */
