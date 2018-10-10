/*
 * ledDriver.h
 *
 *  Created on: 16 мая 2018 г.
 *      Author: opex
 */

#ifndef LEDDRIVER_H_
#define LEDDRIVER_H_

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"



#define cs_set() 		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET)
#define cs_reset() 		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET)
#define cs_strob() 		cs_reset();cs_set()
#define SEGMENT_OFF 	numbers[10]

typedef struct  {

	uint8_t firstReg;
	uint8_t secondReg;
	uint8_t thirdReg;
	uint8_t fourthReg;
	uint8_t LedState;
	uint16_t ShowDelay;

} dysplayBufferStruct;


void getLedBufferFromString(const char* Text, dysplayBufferStruct* ledBuffer);
uint8_t getNumberFromchar(char text);
void getLedBufferFromNumber(uint32_t numberIn, dysplayBufferStruct* ledBuffer);
void getLedBufferFromSpeed(const char* Text, dysplayBufferStruct* ledBuffer);
void getLedBufferFromTime(uint32_t time, dysplayBufferStruct* ledBuffer);
#endif /* LEDDRIVER_H_ */
