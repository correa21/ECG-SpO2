/*
 * externalLED.h
 *
 *  Created on: Oct 6, 2019
 *      Author: armando
 */

#ifndef EXTERNALLED_H_
#define EXTERNALLED_H_

#include "GPIO.h"
#include "PIT.h"
#include "NVIC.h"

/********************************************************************************************/
/*!
	 \brief	 This function configures all the necessary clocks and ports to be used
	 \ in function of the GPIO Port and pin parameters
	 \param[in]  gpio_port_name_t PORT to be used
	 \param[in]  uint8_t		  PIN of the selected port
	 \return void
 */
void LED_set(gpio_port_name_t portName, uint8_t pin);

/********************************************************************************************/
/*!
	 \brief	 This function sets the PIN selected to logical 0
	 \param[in]  gpio_port_name_t PORT to be used
	 \param[in]  uint8_t		  PIN of the selected port
	 \return void
 */
void LED_off(gpio_port_name_t portName, uint8_t pin);

/********************************************************************************************/
/*!
	 \brief	 This function sets the PIN selected to logical 1
	 \param[in]  gpio_port_name_t PORT to be used
	 \param[in]  uint8_t		  PIN of the selected port
	 \return void
 */
void LED_on(gpio_port_name_t portName, uint8_t pin);

/********************************************************************************************/
/*!
	 \brief	 This function configures a PIT timer of 1 second interrupt linked to a pin
	 	 	 port out based on the parameters used
	 \param[in]  uint8_t  times that the led will blink whitin 1 second
	 \param[in]  gpio_port_name_t PORT to be used
	 \param[in]  uint8_t		  PIN of the selected port
	 \return void
 */
void LED_blink(uint8_t times, gpio_port_name_t portName, uint8_t pin);

/********************************************************************************************/
/*!
 	 \brief	 callback function from the pit, toggles the led state and verifies the number
 	 	 	 of times that the led has blinked
 	 \param[in]  void
 	 \return void
 */
void LED_blinking_times(void);

#endif /* EXTERNALLED_H_ */
