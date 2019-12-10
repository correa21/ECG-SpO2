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
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This is a HP recursive filter. it returns the sample without DC component.

 	 \param[in] float sample, float Previous_w
 	 \return float
 */
float MAX30100_DCRemoval(float sample, float Previous_w);

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This is a Mean Different filter. it smooth the input signal.

 	 \param[in] float M, MAX30100_FILTER_t* sensor
 	 \return float
 */
float MAX30100_meanDiff_Filter(float M, MAX30100_FILTER_t* sensor);

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This is a LP BW filter.

 	 \param[in] float x
 	 \return float
 */
float MAX30100_BWLPFilter(float x);

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function returns the actual W from the DC removal for the recursivity.

 	 \param[in] void
 	 \return float
 */
float getDCW(void);

#endif /* MAX30100_FILTERS_H_ */
