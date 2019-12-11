
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
#include "externalLED.h"
#include "times.h"

/**This is mail box to received the information from the serial port*/

#define BEAT_LED_PIN 			bit_1
#define BEAT_LED_PORT 			GPIO_A

#define	REPORTING_PERIOD_MS		5000

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
void bpm_set_LCD_value(float bpm);
uint32_t last_beat_g = 0;
BooleanType is_clean_g = FALSE;
BooleanType is_mainclean_g = TRUE;
BooleanType sampling_g = FALSE;

void beat (void)
{

	static BooleanType led_on = FALSE;
	float SpO2_value = 0;
	float bpm_value = 0;
	SpO2_value = MAX30100_pulseOximeter_getSpO2();
	bpm_value = (MAX30100_pulseOximeter_getHeartRate());
	if (is_clean_g == FALSE)
	{
		LCD_nokia_clear();
		is_clean_g = TRUE;
		is_mainclean_g = FALSE;
	}
	uint8_t string[] = "SpO2";
	LCD_nokia_goto_xy(15,1);
	LCD_nokia_send_string(string);

	uint8_t string1[] = "Bpm";
	LCD_nokia_goto_xy(15,3);
	LCD_nokia_send_string(string1);

	//printf("HR: %d\n", (uint32_t)MAX30100_pulseOximeter_getHeartRate());
	SpO2_set_LCD_value(SpO2_value);
	bpm_set_LCD_value(bpm_value);
	last_beat_g = millis();

	if(TRUE == led_on)
	{
		LED_on(BEAT_LED_PORT,BEAT_LED_PIN);
		led_on = FALSE;
	}
	else
	{
		LED_off(BEAT_LED_PORT,BEAT_LED_PIN);
		led_on = TRUE;
	}

	sampling_g = TRUE;
};

int main(void)
{
	LED_set(BEAT_LED_PORT,BEAT_LED_PIN);
	SPI_init(&g_spi_config); /*! Configuration function for the LCD port*/
	LCD_nokia_init(); /*! Configuration function for the LCD */
	LCD_nokia_clear();
	LED_on(BEAT_LED_PORT,BEAT_LED_PIN);
	MAX30100_pulseOximeter_begin();
	MAX30100_pulseOximeter_setOnBeatDetectedCallback(beat);
	LCD_nokia_bitmap(image_return_address(ITESO_i));

	for(;;)
	{
			MAX30100_pulseOximeter_update();
			if(millis() - last_beat_g > REPORTING_PERIOD_MS)
			{
				sampling_g = FALSE;
				LED_on(BEAT_LED_PORT,BEAT_LED_PIN);
			}
			if (sampling_g == FALSE)
			{
				if(is_mainclean_g == FALSE)
				{
					LCD_nokia_clear();
					is_mainclean_g = TRUE;
					is_clean_g = FALSE;
				}
				LCD_nokia_bitmap(image_return_address(ITESO_i));
			}



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
		LCD_nokia_send_char(decenas+ASCII_0); /** It prints a character   ASCII char '0' starts in 48dec*/
		LCD_nokia_send_char(unidades+ASCII_0); /** It prints a character*/
		LCD_nokia_send_char('.'); /** It prints a character*/
		LCD_nokia_send_char(decimal_1+ASCII_0); /** It prints a character*/
		LCD_nokia_send_char(decimal_2+ASCII_0); /** It prints a character*/
		//LCD_nokia_send_string(string);
	}
	else
	{
		LCD_nokia_goto_xy(15,2);
		LCD_nokia_send_char(ASCII_0); /** It prints a character   ASCII char '0' starts in 48dec*/
		LCD_nokia_send_char(ASCII_0); /** It prints a character*/
		LCD_nokia_send_char('.'); /** It prints a character*/
		LCD_nokia_send_char(ASCII_0); /** It prints a character*/
		LCD_nokia_send_char(ASCII_0); /** It prints a character*/
	}
}

void bpm_set_LCD_value(float bpm_value)
{

	float decimal_data = 0;
	uint8_t centenas   = 0;
	uint8_t decenas    = 0;
	uint8_t unidades   = 0;
	uint8_t decimal_1  = 0;
	uint8_t decimal_2  = 0;
	if(bpm_value < 1000.0)
	{
		decimal_data = bpm_value;
		centenas  = (uint8_t)bpm_value/100;
		decenas   = (uint8_t)bpm_value/10;
		unidades  = ((uint32_t)(decimal_data))%10;
		decimal_1 = ( (((uint32_t)(decimal_data*10))) )%10;
		decimal_2 = ( (((uint32_t)(decimal_data*100))) )%10;


		LCD_nokia_goto_xy(15,4);
		LCD_nokia_send_char(centenas + ASCII_0);
		LCD_nokia_send_char(decenas  + ASCII_0); /** It prints a character   ASCII char '0' starts in 48dec*/
		LCD_nokia_send_char(unidades + ASCII_0); /** It prints a character*/
		LCD_nokia_send_char('.'); /** It prints a character*/
		LCD_nokia_send_char(decimal_1 + ASCII_0); /** It prints a character*/
		LCD_nokia_send_char(decimal_2 + ASCII_0); /** It prints a character*/
		//LCD_nokia_send_string(string);
	}
	else
	{
		LCD_nokia_goto_xy(15,4);
		LCD_nokia_send_char(ASCII_0); /** It prints a character   ASCII char '0' starts in 48dec*/
		LCD_nokia_send_char(ASCII_0); /** It prints a character*/
		LCD_nokia_send_char(ASCII_0); /** It prints a character*/
		LCD_nokia_send_char('.'); /** It prints a character*/
		LCD_nokia_send_char(ASCII_0); /** It prints a character*/
		LCD_nokia_send_char(ASCII_0); /** It prints a character*/
	}
}
