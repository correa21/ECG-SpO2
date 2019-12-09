
#include "MK64F12.h" /* include peripheral declarations */
#include "NVIC.h"/**NVIC device driver*/
#include "I2C.h"
#include "Delay.h"
#include "stdlib.h"
#include "UART_driver.h"
#include "MAX30100_Registers.h"
#include "ECG_ADC.h"
#include "SPI.h"



uint8_t ECG_samples_status = 0;
uint16_t * Buffer_sampled  = 0;
uint16_t static_buffer[320] = {0};
uint16_t index = 0;
int main(void)
{
	ECG_ADC_Init();
	ECG_ADC_Start_Sample();

	for( ; ; )
	{
		ECG_samples_status = ECG_ADC_check_status();

		if( TRUE == ECG_samples_status)
		{
			ECG_samples_status = 0;
			Buffer_sampled = ECG_ADC_return_samples();

			for(index = 0 ; index < 320 ; index++)
			{
				static_buffer[index] = Buffer_sampled[index];
			}
			index = 0;
			ECG_ADC_Start_Sample();
		}
	}

	return 0;
}
