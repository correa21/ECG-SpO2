/*
 * externalLED.c
 *
 *
 *  Created on: Oct 6, 2019
 *      Author: armando
 */


#include "externalLED.h"


#define BLINK_TIME  (2.0F)
#define CLOCK_RATE  (21000000U)

static uint8_t g_number_of_blinks = 0;
static gpio_port_name_t g_portName = 0;
static uint8_t g_pin = 0;

void LED_set(gpio_port_name_t portName, uint8_t pin)
{
	gpio_pin_control_register_t led_config = GPIO_MUX1;
	GPIO_clock_gating(portName);
	GPIO_pin_control_register(portName, pin, &led_config);
	GPIO_data_direction_pin(portName, GPIO_OUTPUT, pin);
	GPIO_clear_pin(portName, pin);
}

void LED_on(gpio_port_name_t portName, uint8_t pin)
{
	GPIO_set_pin(portName, pin);
}

void LED_off(gpio_port_name_t portName, uint8_t pin)
{
	GPIO_clear_pin(portName, pin);
}

void LED_blink(uint8_t times, gpio_port_name_t portName, uint8_t pin)
{
		PIT_clock_gating();
		PIT_enable();
		PIT_enable_interrupt(PIT_2);
		PIT_callback_init(PIT_2, LED_blinking_times);
		PIT_delay(PIT_2, CLOCK_RATE, BLINK_TIME);
		NVIC_enable_interrupt_and_priotity(PIT_CH2_IRQ, PRIORITY_7);
		NVIC_global_enable_interrupts;
		g_number_of_blinks = (times * 2);
		g_portName = portName;
		g_pin = pin;
		PIT_enable_timer(PIT_2);
}

void LED_blinking_times(void)
{
	/*if(0 < g_number_of_blinks) // verify if the number of blinks is done
	{
		GPIO_toogle_pin(g_portName, g_pin);
		g_number_of_blinks--;
	}
	else
	{
		PIT_stop_timer(PIT_2); // number of blinks done
		if(g_pin == LED_INIT)
		{
			LED_on(g_portName, g_pin);
		}
		else
		{
			LED_off(g_portName, g_pin);
		}
	}*/
}
