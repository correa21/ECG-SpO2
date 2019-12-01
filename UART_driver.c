/**
	\file
	\brief
		This is the header file for the UART device driver.
		It contains the macros and function definition.
	\author Javier Chavez
	\date	05/11/2019
	\todo
		To implement all needed functions
		Implementation Only of UART0 for simply coding
 */


#include "UART_driver.h"

uart_mail_box_t g_mail_box_uart_0;
uart_mail_box_t g_mail_box_uart_4;

void UART0_RX_TX_IRQHandler(uart_channel_t uart_channel) /** for serial terminal interface*/
{

	while (!(UART0->S1 & UART_S1_RDRF_MASK)); /** Check if Serial Data is fully received*/
	g_mail_box_uart_0.flag = TRUE;
	g_mail_box_uart_0.mailBox = UART0->D;

}

void UART4_RX_TX_IRQHandler(uart_channel_t uart_channel) /** for bluetooth interface */
{
	while (!(UART4->S1 & UART_S1_RDRF_MASK));  /** Check if Serial Data is fully received*/
	g_mail_box_uart_4.flag = TRUE;
	g_mail_box_uart_4.mailBox = UART4->D;
}

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 It configures the UART to be used
 	 \param[in]  uartChannel indicates which UART will be used.
 	 \param[in]  systemClk indicates the MCU frequency.
 	 \param[in]  baudRate sets the baud rate to transmit.
 	 \return void
 */
void UART_init(uart_channel_t uart_channel, uint32_t system_clk, uart_baud_rate_t baud_rate)
{
    uint16_t sbr_value =  0;
    uint16_t brfa_value = 0;


    /** Calculate the baud rate modulo divisor, sbr*/
    sbr_value = system_clk / (baud_rate * 16);

    brfa_value = (2 * system_clk / (baud_rate)) - 32 * sbr_value;

    switch(uart_channel)
    {
        case UART_0:
			/** Enable UART0 CLOCK*/
			SIM->SCGC4 |= SIM_SCGC4_UART0_MASK;
			/** Disable UART TX RX before setting. */
			UART0->C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK);

			/** Write the sbr value to the BDH and BDL registers*/
		    UART0->BDH = (UART0->BDH & ~UART_BDH_SBR_MASK) | (uint8_t)(sbr_value >> 8);
		    UART0->BDL = (uint8_t)sbr_value;

		    /** Write the brfa value to the register*/
		    UART0->C4 = (UART0->C4 & ~UART_C4_BRFA_MASK) | (brfa_value & UART_C4_BRFA_MASK);

			/** Enable UART TX RX before setting. */
			UART0->C2 |= (UART_C2_TE_MASK | UART_C2_RE_MASK);

    	break;

        case UART_4:
			/** Enable UART4 CLOCK*/
			SIM->SCGC1 |= SIM_SCGC1_UART4_MASK;
			/** Disable UART TX RX before setting. */
			UART4->C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK);

			/** Write the sbr value to the BDH and BDL registers*/
		    UART4->BDH = (UART4->BDH & ~UART_BDH_SBR_MASK) | (uint8_t)(sbr_value >> 8);
		    UART4->BDL = (uint8_t)sbr_value;

		    /** Write the brfa value to the register*/
		    UART4->C4 = (UART4->C4 & ~UART_C4_BRFA_MASK) | (brfa_value & UART_C4_BRFA_MASK);

			/** Enable UART TX RX before setting. */
			UART4->C2 |= (UART_C2_TE_MASK | UART_C2_RE_MASK);

    	break;

        default: /** Add more UART channels*/

        break;
    }


}

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 enables the RX UART interrupt). This function should include the next sentence:
 	 while (!(UART0_S1 & UART_S1_RDRF_MASK)). It is to guaranty that the incoming data is complete
 	 when reception register is read. For more details see chapter 52 in the kinetis reference manual.
 	 \param[in]  uartChannel indicates the UART channel.
 	 \return void
 */
void UART_interrupt_enable(uart_channel_t uart_channel)
{
	switch(uart_channel)
	{
		case UART_0:
			UART0->C2 |= UART_C2_RIE_MASK;  /**  interrupt or DMA transfer requests enabled */
		break;

		case UART_4:
			UART4->C2 |= UART_C2_RIE_MASK;  /**  interrupt or DMA transfer requests enabled */
		break;

		default: /** Add more UART channels*/
		break;
	}
}

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 It sends one character through the serial port. This function should include the next sentence:
 	 while(!(UART0_S1 & UART_S1_TC_MASK)). It is to guaranty that before to try to transmit a byte, the previous
 	 one was transmitted. In other word, to avoid to transmit data while the UART is busy transmitting information.
 	 \param[in]  uartChannel indicates the UART channel.
 	 \param[in]  character to be transmitted.
 	 \return void
 */

void UART_put_char (uart_channel_t uart_channel, uint8_t character)
{
	switch(uart_channel)
	{
		case UART_0:
			while(!(UART0->S1 & UART_S1_TC_MASK)); /** verify if previous bit was transmitted*/
			UART0->D = character;  /** Put data into UART DATA Register */
		break;

		case UART_4:
			while(!(UART4->S1 & UART_S1_TC_MASK)); /** verify if previous bit was transmitted*/
			UART4->D = character;  /** Put data into UART DATA Register */
		break;

		default: /** Add more UART channels*/
		break;
	}
}
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 It sends a string character through the serial port.
 	 \param[in]  uartChannel indicates the UART channel.
 	 \param[in]  string pointer to the string to be transmitted.
 	 \return void
 */
void UART_put_string(uart_channel_t uart_channel, int8_t* string)
{
	while(NULL_CHAR != *string)
	{
		UART_put_char (uart_channel, *string++);
	}
}

uint8_t UART_get_mailbox_flag(uart_channel_t uart_channel)
{
	switch (uart_channel)
	{
	case UART_0:
		return(g_mail_box_uart_0.flag);
	break;

	case UART_4:
		return(g_mail_box_uart_4.flag);
	break;

	default: /** Add more UART channels*/
	break;
	}
	return(0); /**function never reach this point*/
}

void UART_set_mailbox_flag(uart_channel_t uart_channel, uint8_t status)
{
	switch (uart_channel)
	{
	case UART_0:
		g_mail_box_uart_0.flag = status;
	break;

	case UART_4:
		g_mail_box_uart_4.flag = status;
	break;

	default: /** Add more UART channels*/
	break;
	}

}
uint8_t UART_get_mailbox_data(uart_channel_t uart_channel)
{
	switch (uart_channel)
	{
	case UART_0:
		return(g_mail_box_uart_0.mailBox);
	break;

	case UART_4:
		return(g_mail_box_uart_4.mailBox);
	break;

	default: /** Add more UART channels*/
	break;
	}
	return(0); /**function never reach this point*/
}

void UART_Terminal_Configuration(void)
{
	/**Enables the clock of PortB in order to configures TX and RX of UART peripheral*/
		SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
		/**Configures the pin control register of pin16 in PortB as UART RX*/
		PORTB->PCR[16] = PORT_PCR_MUX(3);
		/**Configures the pin control register of pin16 in PortB as UART TX*/
		PORTB->PCR[17] = PORT_PCR_MUX(3);
		/**Configures UART 0 to transmit/receive at 11520 bauds with a 21 MHz of clock core*/
		UART_init (UART_0,  CORE_CLK, BD_115200);
		//printf("UART is configured");
		/**Enables the UART 0 interrupt*/
		UART_interrupt_enable(UART_0);
		/**Enables the UART 0 interrupt in the NVIC*/
		NVIC_enable_interrupt_and_priotity(UART0_IRQ, PRIORITY_4);
		NVIC_global_enable_interrupts;
		/**The following sentences send strings to PC using the UART_put_string function. Also, the string
		 * is coded with terminal code*/

		/*VT100 command for clearing the screen*/
		UART_put_string(UART_0,"\033[2J");

		/** VT100 command for positioning the cursor in x and y position*/
		UART_put_string(UART_0,"\033[10;10H");
		UART_put_string(UART_0, "SISTEMAS EMBEBIDOS I\r");
		/** VT100 command for positioning the cursor in x and y position*/
		UART_put_string(UART_0,"\033[11;10H");
		UART_put_string(UART_0, "    ITESO\r");
		/** VT100 command for positioning the cursor in x and y position*/
		UART_put_string(UART_0,"\033[12;10H");

}

void UART_BT_Terminal_Configuration(void)
{
	/**Enables the clock of PortB in order to configures TX and RX of UART peripheral*/
	SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
	/**Configures the pin control register of pin14 in PortC as UART RX*/
	PORTC->PCR[14] = PORT_PCR_MUX(3);
	/**Configures the pin control register of pin15 in PortC as UART TX*/
	PORTC->PCR[15] = PORT_PCR_MUX(3);
	/**Configures UART 0 to transmit/receive at 11520 bauds with a 21 MHz of clock core*/
	UART_init (UART_4,  CORE_CLK, BD_115200);
	//printf("UART is configured");
	/**Enables the UART 4 interrupt*/
	UART_interrupt_enable(UART_4);
	/**Enables the UART 0 interrupt in the NVIC*/
	NVIC_enable_interrupt_and_priotity(UART4_IRQ, PRIORITY_4);
	NVIC_global_enable_interrupts;
}


