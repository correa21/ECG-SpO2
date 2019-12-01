
#include "MK64F12.h" /* include peripheral declarations */
#include "NVIC.h"/**NVIC device driver*/
#include "I2C.h"
#include "Delay.h"
#include "stdlib.h"
#include "UART_driver.h"
#include "MAX30100_Registers.h"


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
#define MEAN_FILTER_SIZE	(100U)


struct meanFilter_s
{
	float meanFilter_values[MEAN_FILTER_SIZE];
	uint8_t index;
	float sum;
	uint8_t count;
};

struct DCfilter_s
{
	float processed_IR;
	float processed_Red;
	float Red_W;
	float IR_W;

};

typedef struct MAX30100_S{
	uint16_t rawIR;
	uint16_t rawRed;
	float processed_IR;
	float processed_Red;
	struct meanFilter_s meanFilter;

}MAX30100_t;

float MAX30100_DCRemoval(float sample, float Previous_w)
{
	float w;
	float filtered;
	float alpha = 0.95;
	w = sample + (alpha * Previous_w);
	filtered = w - Previous_w;
	Previous_w = w;
	return(filtered);

};

float MAX30100_meanDiff_Filter(float M, MAX30100_t* sensor)
{

	  float avg = 0;

	  sensor->meanFilter.sum -= sensor->meanFilter.meanFilter_values[sensor->meanFilter.index];
	  sensor->meanFilter.meanFilter_values[sensor->meanFilter.index] = M;
	  sensor->meanFilter.sum += sensor->meanFilter.meanFilter_values[sensor->meanFilter.index];

	  sensor->meanFilter.index++;
	  sensor->meanFilter.index = sensor->meanFilter.index % MEAN_FILTER_SIZE;

	  if(sensor->meanFilter.count < MEAN_FILTER_SIZE)
		 sensor->meanFilter.count++;

	  avg =  sensor->meanFilter.sum / sensor->meanFilter.count;
	  return avg - M;
};

float MAX30100_BWLPFilter(float x)
{
	static float values[2] = {0};
	values[0] = values[1];//la muestra n-1 pasa a ser la n

	values[1] = (0.2452372752527856026 * x) + (0.50952544949442879485 * values[0]);// calculo el valor de la muestra n

	return (values[0] + values[1]);//regreso la suma de las muestras
};


int main(void)
{


 	uint8_t status_flag = 0;
	uint8_t data = 0;
	MAX30100_t rawData;
	uint8_t *str;

	uint8_t dataFromMAX30100;

	UART_init(UART_0,CLK,BD_115200);
	 UART_Terminal_Configuration();

	/*!Configuration of MAX30100 sensor */

	I2C_init(I2C_0, CLK, BR);
	I2C_start();

	I2C_write_byte(MAX30100_WRITE);
	I2C_wait();
	I2C_get_ack();

	I2C_write_byte(0x06);//MODE configuration
	I2C_wait();
	I2C_get_ack();


	//I2C_repeated_start();
	I2C_write_byte(0x03); /** SpO2 mode*/
	I2C_wait();
	I2C_get_ack();
	I2C_stop();
	/* ***************************************************** */


	I2C_start();

	I2C_write_byte(MAX30100_WRITE);
	I2C_wait();
	I2C_get_ack();

	I2C_write_byte(0x07);//SPO2 Configuration
	I2C_wait();
	I2C_get_ack();


	//I2C_repeated_start();
	I2C_write_byte(0x47); /**High resolution 100 samples per second and 1.6ms LED pulse width */
	I2C_wait();
	I2C_get_ack();
	I2C_stop();





	I2C_start();

	I2C_write_byte(MAX30100_WRITE);
	I2C_wait();
	I2C_get_ack();

	I2C_write_byte(0x09);//LED Configuration
	I2C_wait();
	I2C_get_ack();


	//I2C_repeated_start();
	I2C_write_byte(0xFF); /** Max current for leds*/
	I2C_wait();
	I2C_get_ack();
	I2C_stop();



		/*****************FIFO READING****************/

	uint8_t buffer[4];//4 elements of the buffer conforms a sample
	uint8_t buffer_index;

	/*	I2C_start();

		I2C_write_byte(MAX30100_WRITE);
		I2C_wait();
		I2C_get_ack();

		I2C_write_byte(0x02);//FIFO WR ptr
		I2C_wait();
		I2C_get_ack();


		I2C_repeated_start();
		I2C_write_byte(MAX30100_READ);
		I2C_wait();
		I2C_get_ack();

		I2C_tx_rx_mode(RECEIVER);
		uint8_t FIFO_WR_ptr = I2C_read_byte();//save the value of the last  FIFO element.
		I2C_stop();


		//getting the FIFO READ pointer
		I2C_start();

		I2C_write_byte(MAX30100_WRITE);
		I2C_wait();
		I2C_get_ack();

		I2C_write_byte(0x04);//FIFO RD ptr
		I2C_wait();
		I2C_get_ack();


		I2C_repeated_start();
		I2C_write_byte(MAX30100_READ);
		I2C_wait();
		I2C_get_ack();

		I2C_tx_rx_mode(RECEIVER);
		uint8_t FIFO_RD_ptr = I2C_read_byte();//save the value of the first  FIFO element.
		I2C_stop();

		uint8_t NUM_AVAILABLE_SAMPLES =  FIFO_WR_ptr - FIFO_RD_ptr;
		uint8_t buffer_index = 0;*/

	//getting the sample to read,



		I2C_start();

		I2C_write_byte(MAX30100_WRITE);
		I2C_wait();
		I2C_get_ack();

		I2C_write_byte(0x05);//FIFO RD ptr
		I2C_wait();
		I2C_get_ack();


		I2C_repeated_start();
		I2C_write_byte(MAX30100_READ);
		I2C_wait();
		I2C_get_ack();

		I2C_tx_rx_mode(RECEIVER);

		for(buffer_index = 0 ; buffer_index < 4 ; buffer_index++)
		{
			buffer[buffer_index] = I2C_read_byte();
			I2C_nack();
			I2C_wait();

		}
		I2C_stop();

		I2C_tx_rx_mode(TRNSMITTER);


		//saving raw data of the sample
		rawData.rawIR = (buffer[0] << 8) | buffer[1];
		rawData.rawRed = (buffer[2] << 8) | buffer[3];

		rawData.processed_IR = MAX30100_DCRemoval(rawData.rawIR);
		rawData.processed_Red = MAX30100_DCRemoval(rawData.rawRed);
		printf("%f", rawData.processed_IR);

		uint8_t array[100];

	for(;;) {

		/*****************FIFO READING****************/



		/*	I2C_start();

			I2C_write_byte(MAX30100_WRITE);
			I2C_wait();
			I2C_get_ack();

			I2C_write_byte(0x02);//FIFO WR ptr
			I2C_wait();
			I2C_get_ack();


			I2C_repeated_start();
			I2C_write_byte(MAX30100_READ);
			I2C_wait();
			I2C_get_ack();

			I2C_tx_rx_mode(RECEIVER);
			uint8_t FIFO_WR_ptr = I2C_read_byte();//save the value of the last  FIFO element.
			I2C_stop();


			//getting the FIFO READ pointer
			I2C_start();

			I2C_write_byte(MAX30100_WRITE);
			I2C_wait();
			I2C_get_ack();

			I2C_write_byte(0x04);//FIFO RD ptr
			I2C_wait();
			I2C_get_ack();


			I2C_repeated_start();
			I2C_write_byte(MAX30100_READ);
			I2C_wait();
			I2C_get_ack();

			I2C_tx_rx_mode(RECEIVER);
			uint8_t FIFO_RD_ptr = I2C_read_byte();//save the value of the first  FIFO element.
			I2C_stop();

			uint8_t NUM_AVAILABLE_SAMPLES =  FIFO_WR_ptr - FIFO_RD_ptr;
			uint8_t buffer_index = 0;*/

		//getting the sample to read,



			I2C_start();

			I2C_write_byte(MAX30100_WRITE);
			I2C_wait();
			I2C_get_ack();

			I2C_write_byte(0x05);//FIFO RD ptr
			I2C_wait();
			I2C_get_ack();


			I2C_repeated_start();
			I2C_write_byte(MAX30100_READ);
			I2C_wait();
			I2C_get_ack();

			I2C_tx_rx_mode(RECEIVER);

			for(buffer_index = 0 ; buffer_index < 4 ; buffer_index++)
			{
				buffer[buffer_index] = I2C_read_byte();
				I2C_nack();
				I2C_wait();

			}
			I2C_stop();

			I2C_tx_rx_mode(TRNSMITTER);


			//saving raw data of the sample
			rawData.rawIR = (buffer[0] << 8) | buffer[1];
			rawData.rawRed = (buffer[2] << 8) | buffer[3];

			rawData.processed_IR = MAX30100_DCRemoval(rawData.rawIR);
			rawData.processed_Red = MAX30100_DCRemoval(rawData.rawRed);

	}

	return 0;
}
