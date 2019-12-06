
#include "MK64F12.h" /* include peripheral declarations */
#include "NVIC.h"/**NVIC device driver*/
#include "I2C.h"
#include "Delay.h"
#include "stdlib.h"
#include "UART_driver.h"
#include "MAX30100_Registers.h"
#include "MAX30100_pulseOximeter.h"

#define DEBUG

#ifdef DEBUG
	#include "stdio.h"
#endif
/**This is mail box to received the information from the serial port*/

#define MAX30100_WRITE 		0xAE
#define MAX30100_READ  		0xAF

#define MAX30100_SPO2_MODE 	0x07
#define MAX30100_SPO2_CONF	0x44
#define MAX30100_FIFO_WR_PTR 0x02

#define	REPORTING_PERIOD_MS		10
void beat (void)
{

	printf("BEAT!\N SpO2: %d\n", MAX30100_pulseOximeter_getSpO2());
	printf("HR: %d\n", (uint32_t)MAX30100_pulseOximeter_getHeartRate());
};

int main(void)
{
	uint16_t tsLastReport;
	MAX30100_pulseOximeter_begin();
	MAX30100_pulseOximeter_setOnBeatDetectedCallback(beat);

	for(;;)
	{
			MAX30100_pulseOximeter_update();


	}

	return 0;
}
