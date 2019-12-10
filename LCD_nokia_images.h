/*
 * LCD_nokia_images.h
 *
 *  Created on: 25/09/2018
 *      Author: jlpe
 */

#ifndef LCD_NOKIA_IMAGES_H_
#define LCD_NOKIA_IMAGES_H_

#include "stdint.h"

typedef enum{ITESO_i,
	         Led_Zeppellin_i,
			 RATM_i,
			 Bender_i,
			 Metallica_i,
			 Rush_i,
			 Perro_i
}image_select_t;

uint8_t * image_return_address(image_select_t);

#endif /* LCD_NOKIA_IMAGES_H_ */
