/**
	\file
	\brief
		This is the header file for the I2C device driver.
		i.e., this is the application programming interface (API) for the GPIO peripheral.
	\author J. Luis Pizano Escalante, luispizano@iteso.mx & modified by Javier Chavez
	\date	7/09/2018
	\todo
	    This API only works with I2C channel 0 for testing purpose
 */

#include "I2C.h"


/********************************************************************************************/
 /********************************************************************************************/
 /********************************************************************************************/
 /*!
  	 \brief
  	 	 Configures the I2C port based on the input parameters.
  	 	 Also, internally this function configures the GPIO, pin control register and clock gating, to be used as I2C.
  	 	 It is recommended to use pin 2 and 3 of GPIOB.
  	 \param[in] channel It is the channel to be used.
  	 \param[in] systemClock Frequency of the system.
  	 \param[in] baudRate baud rate between MCU and I2C device.
  	 \return void

  */
void I2C_init(i2c_channel_t channel, uint32_t system_clock, uint16_t baud_rate)
{
	/** PTB2 -> I2C0 SCL ALT2 */
	/** PTB3 -> I2C0 SDA ALT2 */
	uint8_t SCL_divider = 0;
	SCL_divider = system_clock / (baud_rate * MULT);

	switch(channel)
	{
	case I2C_0:

		SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
		PORTB->PCR[2] = PORT_PCR_MUX(2);
		PORTB->PCR[3] = PORT_PCR_MUX(2);

		SIM->SCGC4 |= SIM_SCGC4_I2C0_MASK;
		I2C0->F |= I2C_F_ICR(SCL_divider);
		I2C0->F |= I2C_F_MULT(MUL);

		I2C0->C1 |= I2C_C1_IICEN_MASK;

	break;

	default: /** Add more I2C channels*/

	break;
	}

}

 /********************************************************************************************/
 /********************************************************************************************/
 /********************************************************************************************/
 /*!
  	 \brief
  	 	 Indicates the status of the bus regardless of slave or master mode. Internally checks the busy bit int the
  	 	 I2Cx_S register. This bit is set when a START signal is detected and cleared when a STOP signal is detected.
  	 \return This function returns a 0 logic if the bus is idle and returns 1 logic if the bus is busy.

  */
 uint8_t I2C_busy(void)
 {
	 uint8_t busy_flag = 0;
	 busy_flag =  (I2C0->S & I2C_S_BUSY_MASK) >> I2C_S_BUSY_SHIFT;
	 return(busy_flag);
 }

 /********************************************************************************************/
 /********************************************************************************************/
 /********************************************************************************************/
 /*!
  	 \brief
  	 	 It selects between master or slave mode.
  	 \param[in] masterOrSlave If == 1 master mode, if == 0 slave mode.
  	 \return void

  */
 void I2C_mst_or_slv_mode(uint8_t mst_or_slv)
 {
	 if(MASTER ==  mst_or_slv)
	 {
		 I2C0->C1 |= I2C_C1_MST_MASK;
	 }
	 else
	 {
		 I2C0->C1 &= ~I2C_C1_MST_MASK;
	 }
 }
 /********************************************************************************************/
 /********************************************************************************************/
 /********************************************************************************************/
 /*!
  	 \brief
  	 	 It selects between transmitter mode or receiver mode.
  	 \param[in] txOrRx If == 1 transmitter mode, if == 0 slave mode.
  	 \return void

  */
 void I2C_tx_rx_mode(uint8_t tx_or_rx)
 {
	 if(RECEIVER ==  tx_or_rx)
	 {
		 I2C0->C1 &= ~I2C_C1_TX_MASK;
	 }
	 else
	 {
		 I2C0->C1 |= I2C_C1_TX_MASK;
	 }
 }
 /********************************************************************************************/
 /********************************************************************************************/
 /********************************************************************************************/
 /*!
  	 \brief
  	 	 It generates the Not ACKnowledge that is needed when the master reads data.
  	 \return void

  */
 void I2C_nack(void)
 {
	 I2C0->C1 |= I2C_C1_TXAK_MASK; /** No acknowledge signal is sent*/
 }

 void I2C_ack(void)
 {
	 I2C0->C1 &= ~I2C_C1_TXAK_MASK; /* An acknowledge signal is sent to the bus on the following receiving byte */
 }
 /********************************************************************************************/
 /********************************************************************************************/
 /********************************************************************************************/
 /*!
  	 \brief
  	 	 It generates a repeated start that is needed when master reads data.
  	 \return void

  */
 void I2C_repeated_start(void)
 {
	 I2C0->C1 |= I2C_C1_RSTA_MASK; /**  Writing 1 to this bit generates a repeated START condition */
 }

 /********************************************************************************************/
 /********************************************************************************************/
 /********************************************************************************************/
 /*!
  	 \brief
  	 	 It writes the data to be transmitted into the transmission buffer. When you want to
  	 	 write a value into the buffer you need to use this sentence I2C0_D = data. Avoid to use
  	 	 masks like this I2C0_D |= data.
  	 \return void

  */
void I2C_write_byte(uint8_t data)
{
	I2C0->D = data;
}

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief
 	 	 It reads data from the receiving buffer.
 	 \return void

 */
uint8_t  I2C_read_byte(void)
{
	uint8_t data = 0;
	data = I2C0->D;
	return(data);
}

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief
 	 	 Indicates the status of the bus regardless of slave or master mode. Internally checks the interrupt flag of the
 	 	 I2Cx_S register. This bit is set on the following events:
			• One byte transfer, including ACK/NACK bit, completes if FACK is 0. An ACK or NACK is sent on the
			  bus by writing 0 or 1 to TXAK after this bit is set in receive mode.
			• One byte transfer, excluding ACK/NACK bit, completes if FACK is 1.

		This function should be implemented as a blocking function by using  while((I2C0->S & 0x02)== 0);, the bit number 2 of this register must be set.
		The blocking implementation of this function only to reduce the complexity of the lab. However, blocking implementation must be avoided.
 	 \return Void.

 */
void I2C_wait(void)
{
	I2C0->S |= (I2C_S_IICIF_MASK);
	while((I2C0->S & I2C_S_IICIF_MASK) >> I2C_S_IICIF_SHIFT == FALSE);
	//I2C0->S |= I2C_S_IICIF_MASK; /** Clear Interrupt */
}

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief
 	 	 Indicates if the acknowledge was received.
 	 \return This function returns a 0 logic if the acknowledge was received and returns 1 logic if the acknowledge was not received.

 */
uint8_t I2C_get_ack(void)
{
	uint8_t ack_flag = FALSE;
	ack_flag = (I2C0->S  & I2C_S_RXAK_MASK);
	return(ack_flag);
}

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief
 	 	 Generates the start signal. When MST bit is changed from 0 to 1, a START signal is generated
 	 	 on the bus and master mode is selected. Also, inside the function the I2C is
 	 	 change to transmitter mode.
 	 \return void

 */
void I2C_start(void)
{
	I2C0->C1 |= (I2C_C1_MST_MASK | I2C_C1_TX_MASK);  /** Master | Transmit*/
	I2C_mst_or_slv_mode(MASTER);
	I2C_tx_rx_mode(TRNSMITTER);
}

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief
 	 	 Generates the stop signal. When this bit changes from 1 to 0, a STOP signal is generated
 	 	 and the mode of operation changes from master to slave. Also, inside the function the I2C is
 	 	 change to receiver mode.
 	 \return void

 */
void I2C_stop(void)
{

	I2C_mst_or_slv_mode(SLAVE);
	I2C_tx_rx_mode(RECEIVER);
}

