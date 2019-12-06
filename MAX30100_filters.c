/*
 * MAX30100_filters.c
 *
 *  Created on: Dec 3, 2019
 *      Author: armando
 */


#include "MAX30100_filters.h"

float last_w;

float MAX30100_DCRemoval(float sample, float Previous_w)
{
	float w;
	float filtered;
	float alpha = (0.95F);
	w = sample + alpha * Previous_w;
	filtered = w - Previous_w;
	last_w = w;
	return(filtered);

};

float MAX30100_meanDiff_Filter(float M, MAX30100_FILTER_t* sensor)
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


float getDCW(void)
{
	return (last_w);
}
