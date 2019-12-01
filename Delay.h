/*
 * Delay.h
 *
 *  Created on: 17 oct 2019
 *      Author: Javier
 */

#ifndef DELAY_H_
#define DELAY_H_

#include "stdint.h"
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief
 	 	 This function generate a blocking delay.

 	 \param[in]  delay number of cycles to wait.
 	 \return void

 */
void delay(uint32_t delay);

#endif /* DELAY_H_ */
