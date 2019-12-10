
#include "MK64F12.h" /* include peripheral declarations */
#include "NVIC.h"/**NVIC device driver*/
#include "I2C.h"
#include "Delay.h"
#include "stdlib.h"
#include "UART_driver.h"
#include "MAX30100_Registers.h"
#include "MAX30100_pulseOximeter.h"
#include "LCD_nokia.h"
#include "LCD_nokia_images.h"
#include "SPI.h"
#include "times.h"

/**This is mail box to received the information from the serial port*/

#define MAX30100_WRITE 		0xAE
#define MAX30100_READ  		0xAF

#define	REPORTING_PERIOD_MS		30

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
									{GPIO_D, bit_0, bit_1, bit_2, bit_3}
								   };
void SpO2_set_LCD_value(float SpO2_value);
uint32_t last_beat_g = 0;

void beat (void)
{

	float SpO2_value = 0;
	SpO2_value = MAX30100_pulseOximeter_getSpO2();
	//printf("HR: %d\n", (uint32_t)MAX30100_pulseOximeter_getHeartRate());
	SpO2_set_LCD_value(SpO2_value);
	last_beat_g = millis();
};

int main(void)
{
	uint8_t string[] = "SpO2";

	SPI_init(&g_spi_config); /*! Configuration function for the LCD port*/
	LCD_nokia_init(); /*! Configuration function for the LCD */
	LCD_nokia_clear();

	uint16_t tsLastReport;
	MAX30100_pulseOximeter_begin();
	MAX30100_pulseOximeter_setOnBeatDetectedCallback(beat);

	LCD_nokia_goto_xy(15,1);
	LCD_nokia_send_string(string);
	for(;;)
	{
			MAX30100_pulseOximeter_update();
			if(millis() - last_beat_g >= REPORTING_PERIOD_MS)
			{
				LCD_nokia_clear();
				LCD_nokia_bitmap(image_return_address(ITESO_i));
			};


	}

	return 0;
}

void SpO2_set_LCD_value(float SpO2_value)
{

	float decimal_data = 0;
	uint8_t decenas    = 0;
	uint8_t unidades   = 0;
	uint8_t decimal_1  = 0;
	uint8_t decimal_2  = 0;
	if(SpO2_value < 100.0)
	{
		decimal_data = SpO2_value;
		decenas   = (uint8_t)SpO2_value/10;
		unidades  = ((uint32_t)(decimal_data))%10;
		decimal_1 = ( (((uint32_t)(decimal_data*10))) )%10;
		decimal_2 = ( (((uint32_t)(decimal_data*100))) )%10;


		LCD_nokia_goto_xy(15,2);
		LCD_nokia_send_char(decenas+48); /** It prints a character   ASCII char '0' starts in 48dec*/
		LCD_nokia_send_char(unidades+48); /** It prints a character*/
		LCD_nokia_send_char('.'); /** It prints a character*/
		LCD_nokia_send_char(decimal_1+48); /** It prints a character*/
		LCD_nokia_send_char(decimal_2+48); /** It prints a character*/
		//LCD_nokia_send_string(string);
	}
	else
	{
		LCD_nokia_goto_xy(15,2);
		LCD_nokia_send_char(48); /** It prints a character   ASCII char '0' starts in 48dec*/
		LCD_nokia_send_char(48); /** It prints a character*/
		LCD_nokia_send_char('.'); /** It prints a character*/
		LCD_nokia_send_char(48); /** It prints a character*/
		LCD_nokia_send_char(48); /** It prints a character*/
	}
}
