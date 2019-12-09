/*
 * RFID_driver.c
 *
 *  Created on: 8 dic 2019
 *      Author: Javier Chavez
 */


#include "RFID_driver.h"

#define SS_PIN bit_3
#define RESET_PIN bit_4
#define SPI_PORT GPIO_D

Uid_t uid;

const spi_config_t g_spi_config = {
							SPI_DISABLE_FIFO,
							SPI_LOW_POLARITY,
							SPI_LOW_PHASE,
							SPI_MSB,
							SPI_0,
							SPI_MASTER,
							GPIO_MUX2 | GPIO_DSE,
							SPI_BAUD_RATE_8,
							SPI_FSIZE_8,
							{GPIO_D, bit_0, bit_1, bit_2, bit_3} };


void RFID_SPI_init(void)
{
	gpio_pin_control_register_t pin_config = GPIO_MUX1;

	/** configure one GPIO port for SS signal */
	GPIO_clock_gating(SPI_PORT);
	GPIO_pin_control_register(SPI_PORT, SS_PIN, &pin_config);
	GPIO_data_direction_pin(SPI_PORT,  GPIO_OUTPUT ,SS_PIN);
	GPIO_clear_pin(SPI_PORT, SS_PIN);

	/**Also configure one GPIO port for Reset signal */
	GPIO_clock_gating(SPI_PORT);
	GPIO_pin_control_register(SPI_PORT, RESET_PIN, &pin_config);
	GPIO_data_direction_pin(SPI_PORT,  GPIO_OUTPUT ,RESET_PIN);
	GPIO_clear_pin(SPI_PORT, RESET_PIN);


	/** SPI Initial Configuration*/
	SPI_init(&g_spi_config);

}


void RFID_module_init(void)
{
	uint8_t pin_data = 0;
	uint8_t hard_reset = FALSE;

	GPIO_set_pin(SPI_PORT, SS_PIN); /** Set to high SS pin*/

	GPIO_clear_pin(SPI_PORT, RESET_PIN);
	delay(500);
	GPIO_set_pin(SPI_PORT, RESET_PIN);
	delay(1000);


	/*
	GPIO_data_direction_pin(SPI_PORT,  GPIO_INPUT ,RESET_PIN);
	pin_data = GPIO_read_pin(SPI_PORT, RESET_PIN);

	if(FALSE == pin_data)
	{
		GPIO_data_direction_pin(SPI_PORT,  GPIO_OUTPUT ,RESET_PIN);
		GPIO_clear_pin(SPI_PORT, RESET_PIN);
		delay(200);
		GPIO_set_pin(SPI_PORT, RESET_PIN);
		delay(500);

		hard_reset = TRUE;
	}

	if(!hard_reset)
	{
		RFID_module_soft_reset();
	}
    */

	RFID_Transfer_Single_Data(TxModeReg, 0x00);
	RFID_Transfer_Single_Data(RxModeReg, 0x00);
	RFID_Transfer_Single_Data(ModWidthReg, 0x26); /** Reset ModWidhtRegister */

	RFID_Transfer_Single_Data(TxModeReg, 0x80);
	RFID_Transfer_Single_Data(TPrescalerReg, 0xA9);
	RFID_Transfer_Single_Data(TReloadRegH, 0x03);
	RFID_Transfer_Single_Data(TReloadRegL, 0xE8);

	RFID_Transfer_Single_Data(TxASKReg, 0x40);
	RFID_Transfer_Single_Data(ModeReg, 0x3D);

	RFID_Transfer_Single_Data(RFCfgReg, (0x07<<4)); // Set Rx Gain to max

	RFID_Antenna_ON();
}

void RFID_Transfer_Single_Data(uint8_t register_address, uint8_t  write_values)
{

	GPIO_clear_pin(SPI_PORT, SS_PIN); /** Enable Slave */

	SPI_tranference(SPI_0, (register_address & 0x7E)); /** send the slave register address*/
	SPI_tranference(SPI_0, write_values);

	GPIO_set_pin(SPI_PORT, SS_PIN); /** Disable Slave */

}

void RFID_Transfer_Data(uint8_t register_address, uint8_t number_values,  uint8_t * write_values)
{
	uint8_t index_values = 0;

	GPIO_clear_pin(SPI_PORT, SS_PIN); /** Enable Slave */

	SPI_tranference(SPI_0, (register_address & 0x7E)); /** send the slave register address*/

	for(index_values = 0 ; index_values < number_values ; index_values++)    /**IMPORTANT CHECK START STOP CONDITIONS*/
	{
		SPI_tranference(SPI_0, write_values[index_values]);
	}


	GPIO_set_pin(SPI_PORT, SS_PIN); /** Disable Slave */

}

void RFID_Antenna_ON(void)
{
	uint8_t data = 0;

	data = RFID_Read_Register(TxControlReg);

	if( 0x03 != (data & 0x03))
	{
		RFID_Transfer_Single_Data(TxControlReg, (data | 0x03) );
	}
}

uint8_t RFID_Read_Register(uint8_t register_address)
{

	uint8_t data_received = 0;
	GPIO_clear_pin(SPI_PORT, SS_PIN); /** Enable Slave */
	SPI_tranference(SPI_0, (0x80 | register_address)); /** MSB = 1 is for reading*/
	data_received = SPI_tranference(SPI_0, 0x00);
	GPIO_set_pin(SPI_PORT, SS_PIN); /** Enable Slave */

	return (data_received);
}

void PCD_ReadRegister(uint8_t reg, uint8_t count, uint8_t *values, uint8_t rxAlign)
{
  if (count == 0)
  {
	  return;
  }

  uint8_t address = 0x80 | reg;  // MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
  uint8_t index = 0;             // Index in values array.

  GPIO_clear_pin(SPI_PORT, SS_PIN); /** Enable Slave */
  count--;                       // One read is performed outside of the loop

  SPI_tranference(SPI_0, address);   // Tell MFRC522 which address we want to read

  while (index < count)
  {
    if ((index == 0) && rxAlign) // Only update bit positions rxAlign..7 in values[0]
    {
      // Create bit mask for bit positions rxAlign..7
      uint8_t mask = 0;
      for (uint8_t i = rxAlign; i <= 7; i++)
      {
        mask |= (1 << i);
      }

      // Read value and tell that we want to read the same address again.
      uint8_t value = SPI_tranference(SPI_0, address);

      // Apply mask to both current value of values[0] and the new data in value.
      values[0] = (values[index] & ~mask) | (value & mask);
    }
    else
    {
      // Read value and tell that we want to read the same address again.
      values[index] = SPI_tranference(SPI_0, address);
    }

    index++;
  }

  values[index] = SPI_tranference(SPI_0, 0x00); // Read the final byte. Send 0 to stop reading.

  GPIO_set_pin(SPI_PORT, SS_PIN);     /* Release SPI Chip MFRC522 */

}


void RFID_module_soft_reset(void)
{
	RFID_Transfer_Single_Data(CommandReg, PCD_SoftReset);
	delay(10000);
}

uint8_t RFID_Get_PresenceCard(void)
{
	uint8_t buffer_ATQA[2] = {0};
	uint8_t buffer_size = 0x02;
	uint8_t status = 0;

	status = RFID_PICC_RequestA(buffer_ATQA, &buffer_size);

	return((status == STATUS_OK) || (status == STATUS_COLLISION));
}

uint8_t RFID_PICC_RequestA(uint8_t * buffer, uint8_t * buffer_size)
{
	return RFID_PICC_REQA_or_WUPA(PICC_CMD_REQA, buffer, buffer_size);
}

uint8_t RFID_PICC_REQA_or_WUPA(uint8_t command, uint8_t *bufferATQA, uint8_t *bufferSize)
{
	  uint8_t validBits;
	  uint8_t status;

	  if (bufferATQA == 0x00 || *bufferSize < 2)
	  {  // The ATQA response is 2 bytes long.
	    return (STATUS_NO_ROOM);
	  }

	  // ValuesAfterColl=1 => Bits received after collision are cleared.
	  RFID_PCD_ClrRegisterBits(CollReg, 0x80);

	  // For REQA and WUPA we need the short frame format
	  // - transmit only 7 bits of the last (and only) byte. TxLastBits = BitFramingReg[2..0]
	  validBits = 7;

	  status = RFID_PCD_TransceiveData(&command, 1, bufferATQA, bufferSize, &validBits, 0, 0);
	  if (status != STATUS_OK)
	  {
	    return status;
	  }

	  if ((*bufferSize != 2) || (validBits != 0))
	  {   // ATQA must be exactly 16 bits.
	    return STATUS_ERROR;
	  }

	  return STATUS_OK;
}

void RFID_PCD_ClrRegisterBits(uint8_t reg, uint8_t mask)
{
  uint8_t tmp = RFID_Read_Register(reg);
  RFID_Transfer_Single_Data(reg, (tmp & (~mask)));    // clear bit mask
}

uint8_t RFID_PCD_TransceiveData(uint8_t *sendData,
								uint8_t sendLen,
								uint8_t *backData,
								uint8_t *backLen,
								uint8_t *validBits,
								uint8_t rxAlign,
								BooleanType checkCRC)
{
	uint8_t waitIRq = 0x30;    // RxIRq and IdleIRq
	uint8_t result = 0;
	result = RFID_PCD_CommunicateWithPICC(PCD_Transceive, waitIRq, sendData, sendLen, backData, backLen, validBits, rxAlign, checkCRC);
	return (result);
}

uint8_t FID_PCD_CommunicateWithPICC(uint8_t command,
							 uint8_t waitIRq,
							 uint8_t *sendData,
							 uint8_t sendLen,
							 uint8_t *backData,
							 uint8_t *backLen,
							 uint8_t *validBits,
							 uint8_t rxAlign,
							 BooleanType    checkCRC)
{
	uint8_t n, _validBits = 0;
	uint32_t i;

	// Prepare values for BitFramingReg
	uint8_t txLastBits = validBits ? *validBits : 0;
	uint8_t bitFraming = (rxAlign << 4) + txLastBits;   // RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]

	RFID_Transfer_Single_Data(CommandReg, PCD_Idle);            // Stop any active command.
	RFID_Transfer_Single_Data(ComIrqReg, 0x7F);                 // Clear all seven interrupt request bits
	RFID_Transfer_Single_Data(FIFOLevelReg, 0x80);            // FlushBuffer = 1, FIFO initialization
	RFID_Transfer_Data(FIFODataReg, sendLen, sendData);  // Write sendData to the FIFO
	RFID_Transfer_Single_Data(BitFramingReg, bitFraming);       // Bit adjustments
	RFID_Transfer_Single_Data(CommandReg, command);             // Execute the command
	if (command == PCD_Transceive)
	{
		PCD_SetRegisterBits(BitFramingReg, 0x80);      // StartSend=1, transmission of data starts
	}

	// Wait for the command to complete.
	// In PCD_Init() we set the TAuto flag in TModeReg. This means the timer automatically starts when the PCD stops transmitting.
	// Each iteration of the do-while-loop takes 17.86us.
	i = 2000;
	while (1)
	{
		n = RFID_Read_Register(ComIrqReg);  // ComIrqReg[7..0] bits are: Set1 TxIRq RxIRq IdleIRq   HiAlertIRq LoAlertIRq ErrIRq TimerIRq
		if (n & waitIRq)
		{          // One of the interrupts that signal success has been set.
			break;
		}

		if (n & 0x01)
		{           // Timer interrupt - nothing received in 25ms
			return STATUS_TIMEOUT;
		}

		if (--i == 0)
		{           // The emergency break. If all other condions fail we will eventually terminate on this one after 35.7ms. Communication with the MFRC522 might be down.
			return STATUS_TIMEOUT;
		}
	}

	// Stop now if any errors except collisions were detected.
	uint8_t errorRegValue = RFID_Read_Register(ErrorReg); // ErrorReg[7..0] bits are: WrErr TempErr reserved BufferOvfl   CollErr CRCErr ParityErr ProtocolErr
	if (errorRegValue & 0x13)
	{  // BufferOvfl ParityErr ProtocolErr
		return STATUS_ERROR;
	}

	// If the caller wants data back, get it from the MFRC522.
	if (backData && backLen)
	{
		n = RFID_Read_Register(FIFOLevelReg);           // Number of bytes in the FIFO
		if (n > *backLen)
		{
			return STATUS_NO_ROOM;
		}

		*backLen = n;                       // Number of bytes returned
		PCD_ReadRegister(FIFODataReg, n, backData, rxAlign);    // Get received data from FIFO
		_validBits = RFID_Read_Register(ControlReg) & 0x07; // RxLastBits[2:0] indicates the number of valid bits in the last received byte. If this value is 000b, the whole byte is valid.
		if (validBits)
		{
			*validBits = _validBits;
		}
	}

	// Tell about collisions
	if (errorRegValue & 0x08)
	{ // CollErr
		return STATUS_COLLISION;
	}

	// Perform CRC_A validation if requested.
	if (backData && backLen && checkCRC)
	{
		// In this case a MIFARE Classic NAK is not OK.
		if ((*backLen == 1) && (_validBits == 4))
		{
			return STATUS_MIFARE_NACK;
		}

		// We need at least the CRC_A value and all 8 bits of the last byte must be received.
		if ((*backLen < 2) || (_validBits != 0))
		{
			return STATUS_CRC_WRONG;
		}

		// Verify CRC_A - do our own calculation and store the control in controlBuffer.
		uint8_t controlBuffer[2];
		n = PCD_CalculateCRC(&backData[0], *backLen - 2, &controlBuffer[0]);
		if (n != STATUS_OK)
		{
			return n;
		}

		if ((backData[*backLen - 2] != controlBuffer[0]) || (backData[*backLen - 1] != controlBuffer[1]))
		{
			return STATUS_CRC_WRONG;
		}
	}

	return STATUS_OK;
}

uint8_t PCD_CalculateCRC(uint8_t *data, uint8_t length, uint8_t *result)
{
	RFID_Transfer_Single_Data(CommandReg, PCD_Idle);      // Stop any active command.
	RFID_Transfer_Single_Data(DivIrqReg, 0x04);           // Clear the CRCIRq interrupt request bit
	RFID_Transfer_Single_Data(FIFOLevelReg, 0x80);      // FlushBuffer = 1, FIFO initialization
	RFID_Transfer_Data(FIFODataReg, length, data); // Write data to the FIFO
	RFID_Transfer_Single_Data(CommandReg, PCD_CalcCRC);   // Start the calculation

	  // Wait for the CRC calculation to complete. Each iteration of the while-loop takes 17.73us.
	  uint16_t i = 5000;
	  uint8_t n;
	  while (1)
	  {
		n = RFID_Read_Register(DivIrqReg);  // DivIrqReg[7..0] bits are: Set2 reserved reserved MfinActIRq   reserved CRCIRq reserved reserved
		if (n & 0x04)
		{
		  // CRCIRq bit set - calculation done
		  break;
		}

		if (--i == 0)
		{
		  // The emergency break. We will eventually terminate on this one after 89ms.
		  // Communication with the MFRC522 might be down.
		  return STATUS_TIMEOUT;
		}
	  }

	  // Stop calculating CRC for new content in the FIFO.
	  RFID_Transfer_Single_Data(CommandReg, PCD_Idle);

	  // Transfer the result from the registers to the result buffer
	  result[0] = RFID_Read_Register(CRCResultRegL);
	  result[1] = RFID_Read_Register(CRCResultRegH);
	  return STATUS_OK;
}

void PCD_SetRegisterBits(uint8_t reg, uint8_t mask)
{
  uint8_t tmp = RFID_Read_Register(reg);
  RFID_Transfer_Single_Data(reg, (tmp | mask));     // set bit mask
}

uint8_t PICC_Select(Uid_t *uid, uint8_t validBits)
{
  BooleanType uidComplete;
  BooleanType selectDone;
  BooleanType useCascadeTag;
  uint8_t cascadeLevel = 1;
  uint8_t result;
  uint8_t count;
  uint8_t index;
  uint8_t uidIndex;          // The first index in uid->uidByte[] that is used in the current Cascade Level.
  uint8_t currentLevelKnownBits;   // The number of known UID bits in the current Cascade Level.
  uint8_t buffer[9];         // The SELECT/ANTICOLLISION commands uses a 7 byte standard frame + 2 bytes CRC_A
  uint8_t bufferUsed;        // The number of bytes used in the buffer, ie the number of bytes to transfer to the FIFO.
  uint8_t rxAlign;           // Used in BitFramingReg. Defines the bit position for the first bit received.
  uint8_t txLastBits;        // Used in BitFramingReg. The number of valid bits in the last transmitted byte.
  uint8_t *responseBuffer;
  uint8_t responseLength;

  // Description of buffer structure:
  //    Byte 0: SEL         Indicates the Cascade Level: PICC_CMD_SEL_CL1, PICC_CMD_SEL_CL2 or PICC_CMD_SEL_CL3
  //    Byte 1: NVB         Number of Valid Bits (in complete command, not just the UID): High nibble: complete bytes, Low nibble: Extra bits.
  //    Byte 2: UID-data or CT    See explanation below. CT means Cascade Tag.
  //    Byte 3: UID-data
  //    Byte 4: UID-data
  //    Byte 5: UID-data
  //    Byte 6: BCC         Block Check Character - XOR of bytes 2-5
  //    Byte 7: CRC_A
  //    Byte 8: CRC_A
  // The BCC and CRC_A is only transmitted if we know all the UID bits of the current Cascade Level.
  //
  // Description of bytes 2-5: (Section 6.5.4 of the ISO/IEC 14443-3 draft: UID contents and cascade levels)
  //    UID size  Cascade level Byte2 Byte3 Byte4 Byte5
  //    ========  ============= ===== ===== ===== =====
  //     4 bytes    1     uid0  uid1  uid2  uid3
  //     7 bytes    1     CT    uid0  uid1  uid2
  //                2     uid3  uid4  uid5  uid6
  //    10 bytes    1     CT    uid0  uid1  uid2
  //                2     CT    uid3  uid4  uid5
  //                3     uid6  uid7  uid8  uid9

  // Sanity checks
  if (validBits > 80)
  {
    return STATUS_INVALID;
  }

  // Prepare MFRC522
  // ValuesAfterColl=1 => Bits received after collision are cleared.
  RFID_PCD_ClrRegisterBits(CollReg, 0x80);

  // Repeat Cascade Level loop until we have a complete UID.
  uidComplete = FALSE;
  while ( ! uidComplete)
  {
    // Set the Cascade Level in the SEL byte, find out if we need to use the Cascade Tag in byte 2.
    switch (cascadeLevel)
    {
      case 1:
        buffer[0] = PICC_CMD_SEL_CL1;
        uidIndex = 0;
        useCascadeTag = validBits && (uid->size > 4); // When we know that the UID has more than 4 bytes
        break;

      case 2:
        buffer[0] = PICC_CMD_SEL_CL2;
        uidIndex = 3;
        useCascadeTag = validBits && (uid->size > 7); // When we know that the UID has more than 7 bytes
        break;

      case 3:
        buffer[0] = PICC_CMD_SEL_CL3;
        uidIndex = 6;
        useCascadeTag = FALSE;            // Never used in CL3.
        break;

      default:
        return STATUS_INTERNAL_ERROR;
        //break;
    }

    // How many UID bits are known in this Cascade Level?
    if(validBits > (8 * uidIndex))
    {
      currentLevelKnownBits = validBits - (8 * uidIndex);
    }
    else
    {
      currentLevelKnownBits = 0;
    }

    // Copy the known bits from uid->uidByte[] to buffer[]
    index = 2; // destination index in buffer[]
    if (useCascadeTag)
    {
      buffer[index++] = PICC_CMD_CT;
    }

    uint8_t bytesToCopy = currentLevelKnownBits / 8 + (currentLevelKnownBits % 8 ? 1 : 0); // The number of bytes needed to represent the known bits for this level.
    if (bytesToCopy)
    {
      // Max 4 bytes in each Cascade Level. Only 3 left if we use the Cascade Tag
      uint8_t maxBytes = useCascadeTag ? 3 : 4;
      if (bytesToCopy > maxBytes)
      {
        bytesToCopy = maxBytes;
      }

      for (count = 0; count < bytesToCopy; count++)
      {
        buffer[index++] = uid->uidByte[uidIndex + count];
      }
    }

    // Now that the data has been copied we need to include the 8 bits in CT in currentLevelKnownBits
    if (useCascadeTag)
    {
      currentLevelKnownBits += 8;
    }

    // Repeat anti collision loop until we can transmit all UID bits + BCC and receive a SAK - max 32 iterations.
    selectDone = FALSE;
    while ( ! selectDone)
    {
      // Find out how many bits and bytes to send and receive.
      if (currentLevelKnownBits >= 32)
      { // All UID bits in this Cascade Level are known. This is a SELECT.
        //Serial.print("SELECT: currentLevelKnownBits="); Serial.println(currentLevelKnownBits, DEC);
        buffer[1] = 0x70; // NVB - Number of Valid Bits: Seven whole bytes

        // Calulate BCC - Block Check Character
        buffer[6] = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5];

        // Calculate CRC_A
        result = PCD_CalculateCRC(buffer, 7, &buffer[7]);
        if (result != STATUS_OK)
        {
          return result;
        }

        txLastBits      = 0; // 0 => All 8 bits are valid.
        bufferUsed      = 9;

        // Store response in the last 3 bytes of buffer (BCC and CRC_A - not needed after tx)
        responseBuffer  = &buffer[6];
        responseLength  = 3;
      }
      else
      { // This is an ANTICOLLISION.
        //Serial.print("ANTICOLLISION: currentLevelKnownBits="); Serial.println(currentLevelKnownBits, DEC);
        txLastBits     = currentLevelKnownBits % 8;
        count          = currentLevelKnownBits / 8;  // Number of whole bytes in the UID part.
        index          = 2 + count;                  // Number of whole bytes: SEL + NVB + UIDs
        buffer[1]      = (index << 4) + txLastBits;  // NVB - Number of Valid Bits
        bufferUsed     = index + (txLastBits ? 1 : 0);

        // Store response in the unused part of buffer
        responseBuffer = &buffer[index];
        responseLength = sizeof(buffer) - index;
      }

      // Set bit adjustments
      rxAlign = txLastBits;                     // Having a seperate variable is overkill. But it makes the next line easier to read.
      RFID_Transfer_Single_Data(BitFramingReg, (rxAlign << 4) + txLastBits);  // RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]

      // Transmit the buffer and receive the response.
      result = RFID_PCD_TransceiveData(buffer, bufferUsed, responseBuffer, &responseLength, &txLastBits, rxAlign, 0);
      if (result == STATUS_COLLISION)
      { // More than one PICC in the field => collision.
        result = RFID_Read_Register(CollReg);     // CollReg[7..0] bits are: ValuesAfterColl reserved CollPosNotValid CollPos[4:0]
        if (result & 0x20)
        { // CollPosNotValid
          return STATUS_COLLISION; // Without a valid collision position we cannot continue
        }

        uint8_t collisionPos = result & 0x1F; // Values 0-31, 0 means bit 32.
        if (collisionPos == 0)
        {
          collisionPos = 32;
        }

        if (collisionPos <= currentLevelKnownBits)
        { // No progress - should not happen
          return STATUS_INTERNAL_ERROR;
        }

        // Choose the PICC with the bit set.
        currentLevelKnownBits = collisionPos;
        count          = (currentLevelKnownBits - 1) % 8; // The bit to modify
        index          = 1 + (currentLevelKnownBits / 8) + (count ? 1 : 0); // First byte is index 0.
        buffer[index] |= (1 << count);
      }
      else if (result != STATUS_OK)
      {
        return result;
      }
      else
      { // STATUS_OK
        if (currentLevelKnownBits >= 32)
        { // This was a SELECT.
          selectDone = TRUE; // No more anticollision
          // We continue below outside the while.
        }
        else
        { // This was an ANTICOLLISION.
          // We now have all 32 bits of the UID in this Cascade Level
          currentLevelKnownBits = 32;
          // Run loop again to do the SELECT.
        }
      }
    } // End of while ( ! selectDone)

    // We do not check the CBB - it was constructed by us above.

    // Copy the found UID bytes from buffer[] to uid->uidByte[]
    index       = (buffer[2] == PICC_CMD_CT) ? 3 : 2; // source index in buffer[]
    bytesToCopy = (buffer[2] == PICC_CMD_CT) ? 3 : 4;
    for (count = 0; count < bytesToCopy; count++)
    {
      uid->uidByte[uidIndex + count] = buffer[index++];
    }

    // Check response SAK (Select Acknowledge)
    if (responseLength != 3 || txLastBits != 0)
    {   // SAK must be exactly 24 bits (1 byte + CRC_A).
      return STATUS_ERROR;
    }

    // Verify CRC_A - do our own calculation and store the control in buffer[2..3] - those bytes are not needed anymore.
    result = PCD_CalculateCRC(responseBuffer, 1, &buffer[2]);
    if (result != STATUS_OK)
    {
      return result;
    }

    if ((buffer[2] != responseBuffer[1]) || (buffer[3] != responseBuffer[2]))
    {
      return STATUS_CRC_WRONG;
    }

    if (responseBuffer[0] & 0x04)
    { // Cascade bit set - UID not complete yes
      cascadeLevel++;
    }
    else
    {
      uidComplete = TRUE;
      uid->sak = responseBuffer[0];
    }
  } // End of while ( ! uidComplete)

  // Set correct uid->size
  uid->size = 3 * cascadeLevel + 1;

  return STATUS_OK;
}

uint8_t PICC_HaltA()
{
  uint8_t result;
  uint8_t buffer[4];

  // Build command buffer
  buffer[0] = PICC_CMD_HLTA;
  buffer[1] = 0;

  // Calculate CRC_A
  result = PCD_CalculateCRC(buffer, 2, &buffer[2]);
  if (result == STATUS_OK)
  {
    // Send the command.
    // The standard says:
    //    If the PICC responds with any modulation during a period of 1 ms after the end of the frame containing the
    //    HLTA command, this response shall be interpreted as 'not acknowledge'.
    // We interpret that this way: Only STATUS_TIMEOUT is an success.
    result = RFID_PCD_TransceiveData(buffer, sizeof(buffer), 0, 0, 0, 0, 0);
    if (result == STATUS_TIMEOUT)
    {
      result = STATUS_OK;
    }
    else if (result == STATUS_OK)
    { // That is ironically NOT ok in this case ;-)
      result = STATUS_ERROR;
    }
  }

  return result;
}

BooleanType PICC_ReadCardSerial(void)
{
  uint8_t result = PICC_Select(&uid, 0);
  return (result == STATUS_OK);
} //
