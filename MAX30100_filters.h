/*
 * MAX30100_filters.h
 *
 *  Created on: Dec 3, 2019
 *      Author: armando
 */

#ifndef MAX30100_FILTERS_H_
#define MAX30100_FILTERS_H_

#include <stdint.h>

#define MEAN_FILTER_SIZE	(15U)



struct meanFilter_s
{
	float meanFilter_values[MEAN_FILTER_SIZE];
	uint8_t index;
	float sum;
	uint8_t count;
};


typedef struct MAX30100_S{
	float Red_W;
	float IR_W;
	struct meanFilter_s meanFilter;

}MAX30100_FILTER_t;

float MAX30100_DCRemoval(float sample, float Previous_w);

float MAX30100_meanDiff_Filter(float M, MAX30100_FILTER_t* sensor);

float MAX30100_BWLPFilter(float x);

float getDCW(void);

#endif /* MAX30100_FILTERS_H_ */
