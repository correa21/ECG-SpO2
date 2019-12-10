/*
 * MAX30100.c
 *
 *  Created on: Nov 30, 2019
 *      Author: armando
 */


#include "MAX30100.h"
#include "stdio.h"
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

	data = I2C_read_byte();
	I2C_tx_rx_mode(TRNSMITTER);
	return (data);
}

static void burstRead(uint8_t baseAddress, uint8_t *buffer, uint8_t length)//lectura de una muestra, cada muestra tiene 4 datos de 8 bits, los primeros dos datos leidos son información del infrarojo y los otros dos son del rojo
{uint8_t buffer_index;
	//for(buffer_index = 0 ; buffer_index < length ; buffer_index++)
	{
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

	for(buffer_index = 0 ; buffer_index < length-1 ; buffer_index++)/*Reading just 3 elements, the last one must bee read outside the loop*/
	{

		/*Dummy read for first 3 elements*/
		buffer[buffer_index] = I2C_read_byte();
		I2C_ack();
		I2C_wait();
		buffer[buffer_index] = I2C_read_byte();

		//I2C_wait();

	}
	/*Dummy read for first last element elements*/
	buffer[buffer_index] = I2C_read_byte();
	I2C_nack();
	I2C_wait();
	I2C_stop();
	buffer[buffer_index] = I2C_read_byte();

		//I2C_nack();
		//I2C_stop();

	//buffer[buffer_index] = I2C_read_byte();

	I2C_tx_rx_mode(TRNSMITTER);}

};

static void MAX30100_readFifoData()//guarda los datos leidos del sensor en sus respectivs variables de 16bits
{
    uint8_t buffer[4];

    burstRead(MAX30100_REG_FIFO_DATA, buffer, BYTES_PER_SAMPLE);

    // Warning: the values are always left-aligned
    rawIRValue = (buffer[0] << 8) | buffer[1];
    rawRedValue = (buffer[2] << 8) | buffer[3];
}


void MAX30100_init(void)
{
	I2C_init(I2C_0, CLK, BR);;//i2c init

    MAX30100_setMode(DEFAULT_MODE);
    MAX30100_setLedsPulseWidth(DEFAULT_PULSE_WIDTH);
    MAX30100_setSamplingRate(DEFAULT_SAMPLING_RATE);
    MAX30100_setLedsCurrent(DEFAULT_IR_LED_CURRENT, DEFAULT_RED_LED_CURRENT);
    MAX30100_setHighresModeEnabled(TRUE);//enable everithing in high resolution.
};



void MAX30100_setMode(Mode mode)//configura el modo
{
	MAX30100_wRegister(MAX30100_REG_MODE_CONFIGURATION, mode);
}

void MAX30100_setLedsPulseWidth(LEDPulseWidth ledPulseWidth)//configura el pulso de los leds
{
    uint8_t previous = MAX30100_rRegister(MAX30100_REG_SPO2_CONFIGURATION);
    MAX30100_wRegister(MAX30100_REG_SPO2_CONFIGURATION, (previous & 0xfc) | ledPulseWidth);
}

void MAX30100_setLedsCurrent(LEDCurrent irLedCurrent, LEDCurrent redLedCurrent)//configura la corriente de los leds
{
	MAX30100_wRegister(MAX30100_REG_LED_CONFIGURATION, (redLedCurrent << 4 | irLedCurrent));
};

void MAX30100_setSamplingRate(SamplingRate samplingRate)//configura la frecuencia de muestreo
{
    uint8_t previous = MAX30100_rRegister(MAX30100_REG_SPO2_CONFIGURATION);
    MAX30100_wRegister(MAX30100_REG_SPO2_CONFIGURATION, (previous & 0xe3) | (samplingRate << 2));
}

void MAX30100_setHighresModeEnabled(BooleanType enabled)//prende o apaga el modo HD
{
    uint8_t previous = MAX30100_rRegister(MAX30100_REG_SPO2_CONFIGURATION);
    if (enabled) {
    	MAX30100_wRegister(MAX30100_REG_SPO2_CONFIGURATION, previous | MAX30100_SPC_SPO2_HI_RES_EN);
    } else {
    	MAX30100_wRegister(MAX30100_REG_SPO2_CONFIGURATION, previous & ~MAX30100_SPC_SPO2_HI_RES_EN);
    }
}

void MAX30100_update(uint16_t*red, uint16_t* ir )//actualiza los valores raw de IR y Red
{
    MAX30100_readFifoData();
    *red = rawRedValue;
    *ir = rawIRValue;
}

void MAX30100_startTemperatureSampling(void)//abilita la medición de la temperatura
{
    uint8_t modeConfig = MAX30100_rRegister(MAX30100_REG_MODE_CONFIGURATION);
    modeConfig |= MAX30100_MC_TEMP_EN;

    MAX30100_wRegister(MAX30100_REG_MODE_CONFIGURATION, modeConfig);
}

BooleanType MAX30100_isTemperatureReady(void)//regresa 1 cuando está lista la temperatura
{
    return !(MAX30100_rRegister(MAX30100_REG_MODE_CONFIGURATION) & MAX30100_MC_TEMP_EN);
}

float MAX30100_retrieveTemperature(void)//recuperamos el valor de la temperatura
{
    int8_t tempInteger = MAX30100_rRegister(MAX30100_REG_TEMPERATURE_DATA_INT);
    float tempFrac = MAX30100_rRegister(MAX30100_REG_TEMPERATURE_DATA_FRAC);

    return tempFrac * 0.0625 + tempInteger;
}





