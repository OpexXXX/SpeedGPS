/*
 * buzzerDriver.h
 *
 *  Created on: 19 мая 2018 г.
 *      Author: opex
 */

/*
 * ledDriver.h
 *
 *  Created on: 16 мая 2018 г.
 *      Author: opex
 */

#ifndef BUZZERDRIVER_H_
#define BUZZERDRIVER_H_

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

uint32_t getPrescallerForFreq(uint16_t freq);

typedef struct  {
	uint16_t freq;
	uint16_t duration;
	uint16_t volume;
} buzzerStruct;


#endif /* BUZZERDRIVER_H_ */
