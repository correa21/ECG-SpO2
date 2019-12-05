/**
	\file
	\brief
		This source files contains the methods to configure ADC module
		and get the voltage values of the input and displayed in NOKIA LCD
	\author Javier Chavez
	\date	30/10/2019
	\todo
 */


#include <ECG_ADC.h>

uint16_t sample_ECG_buffer[320] = {0};
sampling_status_t g_sampling_state = SAMPLING;
uint8_t samples_ready = FALSE;

static BooleanType g_ON = FALSE;

const ADC_config_t g_adc_config = {
							ADC_0,
							ADC_16BIT,
							BUS_CLK,
							SAMPLE_AVG_16,
							LONG_SAMPLE_6
};

void ECG_ADC_Init(void)
{
	/** ADC configuration */
	ADC_Init(&g_adc_config);
	/** PIT configuration */
	PIT_clock_gating();
	PIT_enable();
	PIT_enable_interrupt(PIT_2);
	PIT_callback_init(PIT_2, ECG_ADC_PIT_Callback);
	PIT_delay(PIT_2, CLOCK_RATE, PIT_SAMPLE);

	NVIC_enable_interrupt_and_priotity(PIT_CH2_IRQ, PRIORITY_5);
	NVIC_global_enable_interrupts;
	g_ON = TRUE;

}

void ECG_ADC_Start_Sample()
{
	samples_ready = FALSE;
	g_sampling_state = SAMPLING;
	PIT_enable_timer(PIT_2);
}

void ECG_ADC_PIT_Callback(void)
{
	ECG_ADC_get_values(ADC_0);
}

void ECG_ADC_get_values(ADC_channel_t adc_channel)
{
	static uint16_t samples = 0;

	switch(g_sampling_state)
	{
		case SAMPLING:
			sample_ECG_buffer[samples] = ADC_getMeasure(adc_channel);
			samples++;

			if(MAX_SAMPLES == samples)					/** Wait to take 10 ADC samples */
			{
				samples = 0;
				samples_ready = TRUE;
			    g_sampling_state = IDLE;
			    PIT_stop_timer(PIT_2);
			}

		break;

		case IDLE:
			/** Wait for the extraction of the buffer*/
		break;

		default:
		break;
	}
}

uint8_t ECG_ADC_check_status(void)
{
	return(samples_ready);
}

uint16_t * ECG_ADC_return_samples(void)
{

	return(&sample_ECG_buffer);
}

///////////////////////////////////////////////////////////////////////////////
///EOF
///////////////////////////////////////////////////////////////////////////////
