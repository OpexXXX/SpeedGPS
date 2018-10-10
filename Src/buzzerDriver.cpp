/*
 * buzzerDriver.c
 *
 *  Created on: 19 мая 2018 г.
 *      Author: opex
 */
#include  "buzzerDriver.h"

uint32_t getPrescallerForFreq(uint16_t freq){

	uint32_t result =( (4000000/10)/freq)/2;
	return result;
			//(4000000/10)/1000 = 400Hz

}
