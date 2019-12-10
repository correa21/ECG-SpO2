/**
	\file
	\brief
		This source files contains the methods to configure SPI instance
		send and receive serial data.
	\author Jose Luis Pizano Escalante modified by Javier Chavez
	\date	30/10/2019
	\todo
 */

#include "SPI.h"

/** it enable the clock module of the SPI by modifying the MDIS bits */
 void SPI_enable(spi_channel_t channel)
 {
	 switch (channel)
	 	{
	 		case SPI_0:
	 			SPI0->MCR &= ~SPI_MCR_MDIS_MASK;
	 		break;

	 		case SPI_1:
	 			SPI1->MCR &= ~SPI_MCR_MDIS_MASK;
	 		break;

	 		case SPI_2:
	 			SPI2->MCR &= ~SPI_MCR_MDIS_MASK;
	 		break;

	 		default:
	 			//nothing to do
	 		break;
	 	}
 }

/** It activate the clock gating */
 void SPI_clk(spi_channel_t channel)
 {
	 switch (channel)
	 	{
	 		case SPI_0:
	 			SIM->SCGC6 |= SIM_SCGC6_SPI0_MASK;
	 		break;

	 		case SPI_1:
	 			SIM->SCGC6 |= SIM_SCGC6_SPI1_MASK;
	 		break;

	 		case SPI_2:
	 			SIM->SCGC3 |= SIM_SCGC3_SPI2_MASK;
	 		break;

	 		default:
	 			//nothing to do
	 		break;
	 	}
 }

/** It configure the SPI as a master or slave depending on the value of masterOrslave */
 void SPI_set_master(spi_channel_t channel, spi_master_t master_or_slave)
 {
	 switch (channel)
	 	{
	 		case SPI_0:

	 			if(SPI_MASTER == master_or_slave)
	 			{
	 				SPI0->MCR |= SPI_MCR_MSTR_MASK; /** Enables master mode */

	 			}
	 			else
	 			{
	 				SPI0->MCR &= ~SPI_MCR_MSTR_MASK; /** Enables slave mode */
	 			}

	 		break;

	 		case SPI_1:

	 			if(SPI_MASTER == master_or_slave)
	 			{
	 				SPI1->MCR |= SPI_MCR_MSTR_MASK; /** Enables master mode */

	 			}
	 			else
	 			{
	 				SPI1->MCR &= ~SPI_MCR_MSTR_MASK; /** Enables slave mode */
	 			}
	 		break;

	 		case SPI_2:
	 			if(SPI_MASTER == master_or_slave)
	 			{
	 				SPI2->MCR |= SPI_MCR_MSTR_MASK; /** Enables master mode */

	 			}
	 			else
	 			{
	 				SPI2->MCR &= ~SPI_MCR_MSTR_MASK; /** Enables slave mode */
	 			}
	 		break;

	 		default:
	 			//nothing to do
	 		break;
	 	}
 }

/** It activate the TX and RX FIFOs of the SPI depending on the value of enableOrdisable */
 void SPI_fifo(spi_channel_t channel, spi_enable_fifo_t enable_or_disable)
 {
	 switch (channel)
	 	{
	 		case SPI_0:

	 			if(SPI_ENABLE_FIFO == enable_or_disable)
	 			{
	 				SPI0->MCR &= ~ SPI_MCR_DIS_RXF_MASK; /** Enables RX FIFO */
	 				SPI0->MCR &= ~ SPI_MCR_DIS_TXF_MASK; /** Enables TX FIFO */
	 			}
	 			else
	 			{
	 				SPI0->MCR |=  SPI_MCR_DIS_RXF_MASK; /** Disables RX FIFO */
	 				SPI0->MCR |=  SPI_MCR_DIS_TXF_MASK; /** Disables TX FIFO */
	 			}

	 		break;

	 		case SPI_1:

	 			if(SPI_ENABLE_FIFO == enable_or_disable)
	 			{
	 				SPI1->MCR &= ~ SPI_MCR_DIS_RXF_MASK; /** Enables RX FIFO */
	 				SPI1->MCR &= ~ SPI_MCR_DIS_TXF_MASK; /** Enables TX FIFO */
	 			}
	 			else
	 			{
	 				SPI1->MCR |=  SPI_MCR_DIS_RXF_MASK; /** Disables RX FIFO */
	 				SPI1->MCR |=  SPI_MCR_DIS_TXF_MASK; /** Disables TX FIFO */
	 			}

	 		break;

	 		case SPI_2:

	 			if(SPI_ENABLE_FIFO == enable_or_disable)
	 			{
	 				SPI2->MCR &= ~ SPI_MCR_DIS_RXF_MASK; /** Enables RX FIFO */
	 				SPI2->MCR &= ~ SPI_MCR_DIS_TXF_MASK; /** Enables TX FIFO */
	 			}
	 			else
	 			{
	 				SPI2->MCR |=  SPI_MCR_DIS_RXF_MASK; /** Disables RX FIFO */
	 				SPI2->MCR |=  SPI_MCR_DIS_TXF_MASK; /** Disables TX FIFO */
	 			}

	 		break;

	 		default:
	 			//nothing to do
	 		break;
	 	}
 }


/** It selects the clock polarity depending on the value of cpol */
 void SPI_clock_polarity(spi_channel_t channel, spi_polarity_t cpol)
 {
	 switch (channel)
	 	{
	 		case SPI_0:

	 			if(SPI_HIGH_POLARITY == cpol)
	 			{
	 				SPI0->CTAR[0] |= SPI_CTAR_CPOL_MASK;  /** The inactive state value of SCK is high. */
	 			}
	 			else
	 			{
	 				SPI0->CTAR[0] &= ~SPI_CTAR_CPOL_MASK;  /** The inactive state value of SCK is low. */
	 			}

	 		break;

	 		case SPI_1:

				if(SPI_HIGH_POLARITY == cpol)
				{
					SPI1->CTAR[0] |= SPI_CTAR_CPOL_MASK;  /** The inactive state value of SCK is high. */
				}
				else
				{
					SPI1->CTAR[0] &= ~SPI_CTAR_CPOL_MASK;  /** The inactive state value of SCK is low. */
				}

	 		break;

	 		case SPI_2:

	 			if(SPI_HIGH_POLARITY == cpol)
	 			{
	 				SPI2->CTAR[0] |= SPI_CTAR_CPOL_MASK;  /** The inactive state value of SCK is high. */
	 			}
	 			else
	 			{
	 				SPI2->CTAR[0] &= ~SPI_CTAR_CPOL_MASK;  /** The inactive state value of SCK is low. */
	 			}

	 		break;

	 		default:
	 			//nothing to do
	 		break;
	 	}
 }


/** It selects the frame size depending on the value of frameSize and the macros that are defined above */
 void SPI_frame_size(spi_channel_t channel, uint32_t frame_size)
 {
	 switch (channel)
	 	{
	 		case SPI_0:
	 			SPI0->CTAR[0] &= ~(SPI_CTAR_FMSZ_MASK);  /** Clean FMZ field with zeros*/
	 			SPI0->CTAR[0] = frame_size ; /** Set frame size required */

	 		break;

	 		case SPI_1:
	 			SPI1->CTAR[0] = SPI_CTAR_FMSZ(frame_size);
	 			//SPI1->CTAR[0] |= frame_size;
	 		break;

	 		case SPI_2:
	 			SPI2->CTAR[0] = SPI_CTAR_FMSZ(frame_size);
	 			//SPI2->CTAR[0] |= frame_size;
	 		break;

	 		default:
	 			//nothing to do
	 		break;
	 	}
 }

/** It selects the clock phase depending on the value of cpha */
 void SPI_clock_phase(spi_channel_t channel, spi_phase_t cpha)
 {
	 switch (channel)
	 	{
	 		case SPI_0:

	 			if(SPI_HIGH_PHASE == cpha)
	 			{
	 				SPI0->CTAR[0] |= SPI_CTAR_CPHA_MASK;  /** .Data is captured on the leading edge of SCK and changed on the following edge. */
	 			}
	 			else
	 			{
	 				SPI0->CTAR[0] &= ~SPI_CTAR_CPHA_MASK;  /** Data is changed on the leading edge of SCK and captured on the following edge. */
	 			}

	 		break;

	 		case SPI_1:

		 			if(SPI_HIGH_PHASE == cpha)
		 			{
		 				SPI1->CTAR[0] |= SPI_CTAR_CPHA_MASK;  /** .Data is captured on the leading edge of SCK and changed on the following edge.*/
		 			}
		 			else
		 			{
		 				SPI1->CTAR[0] &= ~SPI_CTAR_CPHA_MASK;  /** Data is changed on the leading edge of SCK and captured on the following edge. */
		 			}

	 		break;

	 		case SPI_2:

	 			if(SPI_HIGH_PHASE == cpha)
	 			{
	 				SPI2->CTAR[0] |= SPI_CTAR_CPHA_MASK;  /** .Data is captured on the leading edge of SCK and changed on the following edge. */
	 			}
	 			else
	 			{
	 				SPI2->CTAR[0] &= ~SPI_CTAR_CPHA_MASK;  /** Data is changed on the leading edge of SCK and captured on the following edge. */
	 			}

	 		break;

	 		default:
	 			//nothing to do
	 		break;
	 	}
 }


/** It selects the baud rate depending on the value of baudRate and the macros that are defined above */
 void SPI_baud_rate(spi_channel_t channel, uint32_t baud_rate)
 {
	//The baud rate is the frequency of the SCK(Serial clock)
	switch (channel)
	{
		case SPI_0:
			SPI0->CTAR[0] &= ~SPI_CTAR_BR_MASK; 	//put the Baud Rate scaler to 0
			SPI0->CTAR[0] |= SPI_CTAR_BR(baud_rate); 		//load the correct baud rate value
		break;

		case SPI_1:
			SPI1->CTAR[0] &= ~SPI_CTAR_BR_MASK; 	//put the Baud Rate scaler to 0
			SPI1->CTAR[0] |= SPI_CTAR_BR(baud_rate); 		//load the correct baud rate value
		break;

		case SPI_2:
			SPI2->CTAR[0] &= ~SPI_CTAR_BR_MASK; 	//put the Baud Rate scaler to 0
			SPI2->CTAR[0] |= SPI_CTAR_BR(baud_rate); 		//load the correct baud rate value
		break;

		default:
			//nothing to do
		break;
	}
 }


/** It selects if MSB or LSM bits is first transmitted */
 void SPI_msb_first(spi_channel_t channel, spi_lsb_or_msb_t msb)
 {
	 if (SPI_MSB == msb)
	 {
		switch (channel)
		{
			case SPI_0:
				SPI0->CTAR[0] &= ~SPI_CTAR_LSBFE_MASK; //Data is transmited MSB first.
			break;

			case SPI_1:
				SPI1->CTAR[0] &= ~SPI_CTAR_LSBFE_MASK; //Data is transmited MSB first.
			break;

			case SPI_2:
				SPI2->CTAR[0] &= ~SPI_CTAR_LSBFE_MASK; //Data is transmited MSB first.
			break;

			default:
				//nothing to do
			break;
		}
	 }
	 else
	 {
		 switch (channel)
		{
			case SPI_0:
				SPI0->CTAR[0] |= SPI_CTAR_LSBFE_MASK; //Data is transmited LSB first.
			break;

			case SPI_1:
				SPI1->CTAR[0] |= SPI_CTAR_LSBFE_MASK; //Data is transmited LSB first.
			break;

			case SPI_2:
				SPI2->CTAR[0] |= SPI_CTAR_LSBFE_MASK; //Data is transmited LSB first.
			break;

			default:
				//nothing to do
			break;
		}
	 }
 }


/** It stars the SPI transmission by modifying the value of HALT bit */
void SPI_start_tranference(spi_channel_t channel)
{
	switch (channel)
	{
		case SPI_0:
			SPI0->MCR &= ~SPI_MCR_HALT_MASK; //Start transfers
		break;

		case SPI_1:
			SPI1->MCR &= ~SPI_MCR_HALT_MASK; //Start transfers
		break;

		case SPI_2:
			SPI2->MCR &= ~SPI_MCR_HALT_MASK; //Start transfers
		break;

		default:
			//nothing to do
		break;
	}
}

/** It stops the SPI transmission by modifying the value of HALT bit */
void SPI_stop_tranference(spi_channel_t channel)
{
	switch (channel)
	{
		case SPI_0:
			SPI0->MCR |= SPI_MCR_HALT_MASK; //stops transfers
		break;

		case SPI_1:
			SPI1->MCR |= SPI_MCR_HALT_MASK; //stops transfers
		break;

		case SPI_2:
			SPI2->MCR |= SPI_MCR_HALT_MASK; //stops transfers
		break;

		default:
			//nothing to do
		break;
	}
}


/** It transmits the information contained in data */
uint8_t SPI_tranference(spi_channel_t channel, uint8_t data)
{
	uint8_t received_data = 0;
	SPI_start_tranference(channel);

	switch (channel)
	{
		case SPI_0:
			SPI0->PUSHR = (data) | SPI_PUSHR_EOQ_MASK;
			while((SPI0->SR & SPI_SR_TCF_MASK) == 0);
			SPI0->SR |= SPI_SR_TCF_MASK;
			received_data = SPI0->POPR & 0xFF;
		break;

		case SPI_1:
			SPI1->PUSHR = (data) | SPI_PUSHR_EOQ_MASK;
			while((SPI1->SR & SPI_SR_TCF_MASK) == 0);
			SPI1->SR |= SPI_SR_TCF_MASK;
			received_data = SPI1->POPR & 0xFF;
		break;

		case SPI_2:
			SPI2->PUSHR = (data) | SPI_PUSHR_EOQ_MASK;
			while((SPI2->SR & SPI_SR_TCF_MASK) == 0);
			SPI2->SR |= SPI_SR_TCF_MASK;
			received_data = SPI2->POPR & 0xFF;
		break;

		default:
			//nothing to do
		break;
	}
	SPI_stop_tranference(channel);
	return (received_data);
}

/** It configures the SPI for transmission, this function as arguments receives a pointer to a constant structure where are all
 * the configuration parameters */
void SPI_init(const spi_config_t* config_struct)
{
	SPI_clk(config_struct->spi_channel);

	//GPIO port configuration routine

	GPIO_clock_gating(config_struct->spi_gpio_port.gpio_port_name);
	GPIO_pin_control_register(config_struct->spi_gpio_port.gpio_port_name, config_struct->spi_gpio_port.spi_clk, &(config_struct->pin_config));
	GPIO_pin_control_register(config_struct->spi_gpio_port.gpio_port_name, config_struct->spi_gpio_port.spi_sout, &(config_struct->pin_config));
	GPIO_pin_control_register(config_struct->spi_gpio_port.gpio_port_name, config_struct->spi_gpio_port.spi_sin, &(config_struct->pin_config));

	//SPI configuration functions

	SPI_set_master(config_struct->spi_channel, config_struct->spi_master);
	SPI_fifo(config_struct->spi_channel, config_struct->spi_enable_fifo);
	SPI_clock_polarity(config_struct->spi_channel, config_struct->spi_polarity);
	SPI_frame_size(config_struct->spi_channel, config_struct->spi_frame_size);
	SPI_clock_phase(config_struct->spi_channel, config_struct->spi_phase);
	SPI_baud_rate(config_struct->spi_channel, config_struct->spi_baudrate);
	SPI_msb_first(config_struct->spi_channel, config_struct->spi_lsb_or_msb);
	SPI_enable(config_struct->spi_channel);
}
