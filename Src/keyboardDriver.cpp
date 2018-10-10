/*
 * keyboardDriver.c
 *
 *  Created on: 16 мая 2018 г.
 *      Author: opex
 */

#include "keyboardDriver.h"
#include "main.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"



void init_KeyboardObj(){
	prevousPressedKey = NOTHING_KEY;
	countPressTime = 0;
	countAutoPressAgainDelayTime = 0;
	countAgainAxel = 5;
	countAutoPress = 0;
	longPressFlag = 0;
}

uint16_t GPIO_Scan(){

	LINE_1_UP;
	if(READ_LINE_6) return STAR_KEY;//*
	if(READ_LINE_7) return SEVEN_KEY;//7
	if(READ_LINE_8) return THREE_KEY;//3
	if(READ_LINE_10) return DOWN_KEY;//down
	LINE_1_DOWN;
	LINE_2_UP;
	if(READ_LINE_6)return ZERO_KEY ;//0
	if(READ_LINE_7) return SIX_KEY;//6
	if(READ_LINE_8) return TWO_KEY;//2
	if(READ_LINE_10)return UP_KEY ;//up
	LINE_2_DOWN;
	LINE_3_UP;
	if(READ_LINE_6) return NINE_KEY;//9
	if(READ_LINE_7) return FIVE_KEY;//5
	if(READ_LINE_8)return ONE_KEY;//1
	LINE_3_DOWN;
	LINE_4_UP;
	if(READ_LINE_6) return EIGHT_KEY;//8
	if(READ_LINE_7) return FOUR_KEY;//4
	if(READ_LINE_10) return REDIAL_KEY;//red
	if(READ_LINE_5) return RESH_KEY;//#
	LINE_4_DOWN;

	LINE_9_UP;
	if(READ_LINE_5) return FL_KEY ;//flash
	LINE_9_DOWN;

	return NOTHING_KEY;
}

uint16_t scanKeyboard()
{

	uint16_t result = GPIO_Scan();
	ALL_LINE_DOWN;
	return result;
}



pressedKeyStruct  scanKeyboardLoop()
{
	uint16_t scanResult = NOTHING_KEY;
	scanResult = scanKeyboard();
	pressKeyStructForQueue.pressKey = prevousPressedKey;
	if (scanResult == NOTHING_KEY && prevousPressedKey != NOTHING_KEY) {//Обработчики отпускания кнопки
		if ((countPressTime < BUTTON_DELAY_OF_LONG_PRESS)
				&& countAutoPress == 0) {//Обработчики отпускания после короткого нажатия
			prevousPressedKey = scanResult;
			pressKeyStructForQueue.longPress=0;
			return pressKeyStructForQueue;
		}
		if ((countPressTime > BUTTON_DELAY_OF_LONG_PRESS)
				&& countAutoPress == 0) {//Обработчик отпускания после длительного нажатия
			prevousPressedKey = scanResult;
			pressKeyStructForQueue.longPress=1;
			return pressKeyStructForQueue;
		}
		if ((countPressTime > BUTTON_DELAY_OF_START_AUTO_PRESS)
				&& countAutoPress > 0) {//Обработчик отпускания после процедуры автонажатий
		}
		prevousPressedKey = NOTHING_KEY;
		countPressTime = 0;
		countAutoPress = 0;
		countAgainAxel = 0;
		countAutoPressAgainDelayTime = 0;
		longPressFlag = 0;
	}
	if (scanResult != NOTHING_KEY) {
		if (prevousPressedKey != scanResult) {
			countPressTime = 0;
			countAutoPress = 0;
			countAgainAxel = 0;
			countAutoPressAgainDelayTime = 0;
			longPressFlag = 0;
		}
		if (prevousPressedKey == scanResult) {
			countPressTime++;
			if ((countPressTime > BUTTON_DELAY_OF_LONG_PRESS)
					&& countAutoPress == 0 && longPressFlag == 0) {
				longPressFlag = 1;
				/*ОБРАБОТЧИК БУЗЕРА ПРИ ПЕРЕХОДЕ В РЕЖИМ ДЛИТЕЛЬНОЕ НАЖАТИЕ*/
				prevousPressedKey = scanResult;
				pressKeyStructForQueue.longPress=1;
				pressKeyStructForQueue.pressKey= NOTHING_KEY;
			}
		}
		if ((countPressTime) > (BUTTON_DELAY_OF_START_AUTO_PRESS)) {
			countAutoPressAgainDelayTime++;
			if (countAutoPressAgainDelayTime
					> (BUTTON_MAX_DELAY_OF_AUTO_PRESS - countAgainAxel)) {

				countAgainAxel += BUTTON_AXELERATION_AUTO_PRESS;
				countAutoPress++;
				countAutoPressAgainDelayTime = 0;
				if (countAgainAxel
						> (BUTTON_MAX_DELAY_OF_AUTO_PRESS
								- BUTTON_MIN_DELAY_OF_AUTO_PRESS)) {
					countAgainAxel = BUTTON_MAX_DELAY_OF_AUTO_PRESS
							- BUTTON_MIN_DELAY_OF_AUTO_PRESS;
				}
				prevousPressedKey = scanResult;
				pressKeyStructForQueue.longPress=0;
				return pressKeyStructForQueue;
			}
		}
	}
	prevousPressedKey = scanResult;
	pressKeyStructForQueue.pressKey = NOTHING_KEY;
	pressKeyStructForQueue.longPress=0;
	return pressKeyStructForQueue;

}


