/*
 * MAX30100.c
 *
 *  Created on: Nov 30, 2019
 *      Author: armando
 */


#include "MAX30100.h"

uint16_t rawIRValue;
uint16_t rawRedValue;

static void MAX30100_wRegister(uint8_t adress, uint8_t data)
{
	I2C_start();

		I2C_write_byte(MAX30100_WRITE);
		I2C_wait();
		I2C_get_ack();

		I2C_write_byte(adress);
		I2C_wait();
		I2C_get_ack();

		I2C_write_byte(data);
		I2C_wait();
		I2C_get_ack();
		I2C_stop();
};

uint8_t MAX30100_rRegister(uint8_t adress)
{
	I2C_start();

	I2C_write_byte(MAX30100_WRITE);
	I2C_wait();
	I2C_get_ack();

	I2C_write_byte(adress);
	I2C_wait();
	I2C_get_ack();


	I2C_repeated_start();
	I2C_write_byte(MAX30100_READ);
	I2C_wait();
	I2C_get_ack();

	I2C_tx_rx_mode(RECEIVER);

	uint8_t data = I2C_read_byte();
	I2C_nack();
	I2C_wait();

	I2C_stop();

	I2C_tx_rx_mode(TRNSMITTER);
	return (data);
}

static void burstRead(uint8_t baseAddress, uint8_t *buffer, uint8_t length)//lectura de una muestra, cada muestra tiene 4 datos de 8 bits, los primeros dos datos leidos son información del infrarojo y los otros dos son del rojo
{
	uint8_t buffer_index;
	I2C_start();

	I2C_write_byte(MAX30100_WRITE);
	I2C_wait();
	I2C_get_ack();

	I2C_write_byte(baseAddress);//FIFO RD ptr
	I2C_wait();
	I2C_get_ack();


	I2C_repeated_start();
	I2C_write_byte(MAX30100_READ);
	I2C_wait();
	I2C_get_ack();

	I2C_tx_rx_mode(RECEIVER);

	for(buffer_index = 0 ; buffer_index < length ; buffer_index++)
	{
		buffer[buffer_index] = I2C_read_byte();
		I2C_nack();
		I2C_wait();

	}
	I2C_stop();

	I2C_tx_rx_mode(TRNSMITTER);
};

static void MAX30100_readFifoData()//guarda los datos leidos del sensor en sus respectivs variables de 16bits
{
    uint8_t buffer[4];

    burstRead(MAX30100_REG_FIFO_DATA, buffer, BYTES_PER_SAMPLE);

    // Warning: the values are always left-aligned
    rawIRValue = (buffer[0] << 8) | buffer[1];
    rawRedValue = (buffer[2] << 8) | buffer[3];
}

void MAX30100_setMode(Mode mode)//configura el modo
{
    writeRegister(MAX30100_REG_MODE_CONFIGURATION, mode);
}

void MAX30100_setLedsPulseWidth(LEDPulseWidth ledPulseWidth)//configura el pulso de los leds
{
    uint8_t previous = readRegister(MAX30100_REG_SPO2_CONFIGURATION);
    writeRegister(MAX30100_REG_SPO2_CONFIGURATION, (previous & 0xfc) | ledPulseWidth);
}

void MAX30100_setLedsCurrent(LEDCurrent irLedCurrent, LEDCurrent redLedCurrent)//configura la corriente de los leds
{
	MAX30100_wRegister(MAX30100_REG_LED_CONFIGURATION, (redLedCurrent << 4 | irLedCurrent));
};

void MAX30100_setSamplingRate(SamplingRate samplingRate)//configura la frecuencia de muestreo
{
    uint8_t previous = readRegister(MAX30100_REG_SPO2_CONFIGURATION);
    writeRegister(MAX30100_REG_SPO2_CONFIGURATION, (previous & 0xe3) | (samplingRate << 2));
}

void MAX30100_setHighresModeEnabled(BooleanType enabled)//prende o apaga el modo HD
{
    uint8_t previous = readRegister(MAX30100_REG_SPO2_CONFIGURATION);
    if (enabled) {
        writeRegister(MAX30100_REG_SPO2_CONFIGURATION, previous | MAX30100_SPC_SPO2_HI_RES_EN);
    } else {
        writeRegister(MAX30100_REG_SPO2_CONFIGURATION, previous & ~MAX30100_SPC_SPO2_HI_RES_EN);
    }
}

void MAX30100_update()//actualiza los valores raw de IR y Red
{
    MAX30100_readFifoData();
}

void MAX30100_startTemperatureSampling()//abilita la medición de la temperatura
{
    uint8_t modeConfig = readRegister(MAX30100_REG_MODE_CONFIGURATION);
    modeConfig |= MAX30100_MC_TEMP_EN;

    writeRegister(MAX30100_REG_MODE_CONFIGURATION, modeConfig);
}

void MAX30100_isTemperatureReady()//regresa 1 cuando está lista la temperatura
{
    return !(readRegister(MAX30100_REG_MODE_CONFIGURATION) & MAX30100_MC_TEMP_EN);
}

float MAX30100_retrieveTemperature()//recuperamos el valor de la temperatura
{
    int8_t tempInteger = readRegister(MAX30100_REG_TEMPERATURE_DATA_INT);
    float tempFrac = readRegister(MAX30100_REG_TEMPERATURE_DATA_FRAC);

    return tempFrac * 0.0625 + tempInteger;
}



