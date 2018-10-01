/*
 * keyboardDriver.h
 *
 *  Created on: 16 мая 2018 г.
 *      Author: opex
 */

#ifndef KEYBOADRDRIVER_H_
#define KEYBOADRDRIVER_H_

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"


#define LINE_1_UP 				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
#define LINE_1_DOWN  			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
#define LINE_2_UP				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
#define LINE_2_DOWN				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
#define LINE_3_UP				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
#define LINE_3_DOWN				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
#define LINE_4_UP				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
#define LINE_4_DOWN				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
#define LINE_9_UP				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
#define LINE_9_DOWN				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);

#define ALL_LINE_DOWN			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);

#define READ_LINE_5 			HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9)==GPIO_PIN_SET
#define READ_LINE_6 			HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8)==GPIO_PIN_SET
#define READ_LINE_7				HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15)==GPIO_PIN_SET
#define READ_LINE_8				HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14)==GPIO_PIN_SET
#define READ_LINE_10			HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12)==GPIO_PIN_SET

#define BUTTON_DELAY_OF_LONG_PRESS					300
#define BUTTON_DELAY_OF_START_AUTO_PRESS			700
#define BUTTON_MAX_DELAY_OF_AUTO_PRESS				500
#define BUTTON_MIN_DELAY_OF_AUTO_PRESS 				25
#define BUTTON_AXELERATION_AUTO_PRESS 				25

 enum keycode{
	NOTHING_KEY,
	ONE_KEY,
	TWO_KEY,
	THREE_KEY,
	FOUR_KEY,
	FIVE_KEY,
	SIX_KEY,
	SEVEN_KEY,
	EIGHT_KEY,
	NINE_KEY,
	ZERO_KEY,
	FL_KEY,
	REDIAL_KEY,
	UP_KEY,
	DOWN_KEY,
	STAR_KEY,
	RESH_KEY
};

 typedef struct  {
	uint16_t pressKey;
	uint16_t modifiKey;
	uint16_t longPress;
} pressedKeyStruct;

 //static osMessageQId* PressedKeyQueue0;
 //static osMessageQId* BuzzerQueue0;
 static uint16_t prevousPressedKey ;
 static uint32_t countPressTime ;
 static uint32_t countAutoPressAgainDelayTime ;
 static  uint32_t countAgainAxel;
 static  uint32_t countAutoPress ;
 static pressedKeyStruct pressKeyStructForQueue;
 static  uint16_t longPressFlag;
 pressedKeyStruct scanKeyboardLoop(void);
  void init_KeyboardObj(void);
   uint16_t scanKeyboard(void);
  uint16_t GPIO_Scan(void);



#endif /* KEYBOADRDRIVER_H_ */

