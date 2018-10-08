/*
 * ledDriver.c
 *
 *  Created on: 16 мая 2018 г.
 *      Author: opex
 */
#include "ledDriver.h"


uint16_t delay_disp = 10;

uint8_t numbers[12]=
{
		0b00010100, //0
		0b01110111,//1
		0b00011010,//2
		0b01010010,//3
		0b01110001, //4
		0b11010000, //5
		0b10010000, //6
		0b01110110, //7
		0b00010000, //8
		0b01010000,//9
		0b11111111 // Выкл
};



uint8_t getNumberFromchar(char text){
	switch (text)
						{
							case '1':
								return numbers[1];
								break;
							case '2':
								return numbers[2];
								break;
							case '3':
								return numbers[3];
								break;
							case '4':
								return numbers[4];
								break;
							case '5':
								return numbers[5];
								break;
							case '6':
								return numbers[6];
								break;
							case '7':
								return numbers[7];
								break;
							case '8':
								return numbers[8];
								break;
							case '9':
								return numbers[9];
								break;
							case '0':
								return numbers[0];
								break;
							default:
								return numbers[11];
								break;
						}
}
void getLedBufferFromSpeed(const char* Text, dysplayBufferStruct* ledBuffer) {
	uint8_t tempBuf[8]={
			SEGMENT_OFF,
			SEGMENT_OFF,
			SEGMENT_OFF,
			SEGMENT_OFF
	};
	uint8_t counter = 0;                /* счетчик и одновременно индекс      */
	uint8_t counter2 = 0;
	//while(*s1)
	while(*Text){   /* пока не встретился признак конца текста */
		if( *Text!= '.') {
			tempBuf[counter2]=getNumberFromchar(*Text);
			counter2++;
		}
		if( *Text=='.') {
			if(counter2 > 0){
			tempBuf[counter2-1]=tempBuf[counter2-1]&~0b00010000;
			}else{
			tempBuf[counter2]=tempBuf[counter2]|0b00010000;
			}
		}
		Text++;
	}

	if(tempBuf[3]== SEGMENT_OFF){
		tempBuf[3]=tempBuf[2];
		tempBuf[2]=tempBuf[1];
		tempBuf[1]=tempBuf[0];
		tempBuf[0]=SEGMENT_OFF;
	}

	if(tempBuf[3]== SEGMENT_OFF){
		tempBuf[3]=tempBuf[2];
		tempBuf[2]=tempBuf[1];
		tempBuf[1]=tempBuf[0];
		tempBuf[0]=SEGMENT_OFF;
	}

	if(tempBuf[3]==  SEGMENT_OFF){
		tempBuf[3]=tempBuf[2];
		tempBuf[2]=tempBuf[1];
		tempBuf[1]=tempBuf[0];
		tempBuf[0]=SEGMENT_OFF;
	}

	if(tempBuf[3]== SEGMENT_OFF){
		tempBuf[3]=tempBuf[2];
		tempBuf[2]=tempBuf[1];
		tempBuf[1]=tempBuf[0];
		tempBuf[0]=SEGMENT_OFF;
	}


	/*if(tempBuf[2]== SEGMENT_OFF){
			tempBuf[2]=tempBuf[1];
			tempBuf[1]=tempBuf[0];
			tempBuf[0]=SEGMENT_OFF;
		}
	if(tempBuf[1]== SEGMENT_OFF){
			tempBuf[1]=tempBuf[0];
			tempBuf[0]=SEGMENT_OFF;
		}*/



	    ledBuffer->firstReg = tempBuf[3];
		ledBuffer->secondReg = tempBuf[2];
		ledBuffer->thirdReg = tempBuf[1];
		ledBuffer->fourthReg = tempBuf[0];
}

void getLedBufferFromString(const char* Text, dysplayBufferStruct* ledBuffer) {
	uint8_t tempBuf[4]={
			SEGMENT_OFF,
			SEGMENT_OFF,
			SEGMENT_OFF,
			SEGMENT_OFF
	};
	uint8_t counter = 0;                /* счетчик и одновременно индекс      */
	uint8_t counter2 = 0;
	//while(*s1)
	while(*Text || counter2<4){   /* пока не встретился признак конца текста */
		if( *Text!= '.') {
			tempBuf[counter2]=getNumberFromchar(*Text);
			counter2++;
		}
		if( *Text=='.') {
			if(counter2 > 0){
			tempBuf[counter2-1]=tempBuf[counter2-1]&~0b00010000;
			}else{
			tempBuf[counter2]=tempBuf[counter2]|0b00010000;
			}
		}
		Text++;
	}

	if(tempBuf[3]== SEGMENT_OFF){
		tempBuf[3]=tempBuf[2];
		tempBuf[2]=tempBuf[1];
		tempBuf[1]=tempBuf[0];
		tempBuf[0]=SEGMENT_OFF;
	}

	if(tempBuf[3]== SEGMENT_OFF){
		tempBuf[3]=tempBuf[2];
		tempBuf[2]=tempBuf[1];
		tempBuf[1]=tempBuf[0];
		tempBuf[0]=SEGMENT_OFF;
	}

	if(tempBuf[3]==  SEGMENT_OFF){
		tempBuf[3]=tempBuf[2];
		tempBuf[2]=tempBuf[1];
		tempBuf[1]=tempBuf[0];
		tempBuf[0]=SEGMENT_OFF;
	}

	if(tempBuf[3]== SEGMENT_OFF){
		tempBuf[3]=tempBuf[2];
		tempBuf[2]=tempBuf[1];
		tempBuf[1]=tempBuf[0];
		tempBuf[0]=SEGMENT_OFF;
	}


	/*if(tempBuf[2]== SEGMENT_OFF){
			tempBuf[2]=tempBuf[1];
			tempBuf[1]=tempBuf[0];
			tempBuf[0]=SEGMENT_OFF;
		}
	if(tempBuf[1]== SEGMENT_OFF){
			tempBuf[1]=tempBuf[0];
			tempBuf[0]=SEGMENT_OFF;
		}*/



	    ledBuffer->firstReg = tempBuf[3];
		ledBuffer->secondReg = tempBuf[2];
		ledBuffer->thirdReg = tempBuf[1];
		ledBuffer->fourthReg = tempBuf[0];
}
void getLedBufferFromNumber(uint32_t number, dysplayBufferStruct* ledBuffer) {

	ledBuffer->firstReg = numbers[0];
	ledBuffer->secondReg = numbers[0];
	ledBuffer->thirdReg = numbers[0];
	ledBuffer->fourthReg = numbers[0];
	if (number < 10) {
		ledBuffer->firstReg  = numbers[number];
		ledBuffer->secondReg = SEGMENT_OFF;
		ledBuffer->thirdReg = SEGMENT_OFF;
		ledBuffer->fourthReg  = SEGMENT_OFF;
	}
	if (number >= 10 && number < 100) {
		ledBuffer->firstReg  = numbers[number % 10];
		ledBuffer->secondReg = numbers[number / 10];
		ledBuffer->thirdReg = SEGMENT_OFF;
		ledBuffer->fourthReg  = SEGMENT_OFF;
	}
	if (number >= 100 && number < 1000) {
		ledBuffer->firstReg  = numbers[number % 10];
		ledBuffer->secondReg = numbers[number % 100 / 10];
		ledBuffer->thirdReg = numbers[number / 100];
		ledBuffer->fourthReg  = SEGMENT_OFF;
	}
	if (number >= 1000 && number < 10000) {
		ledBuffer->firstReg  = numbers[number % 10];
		ledBuffer->secondReg = numbers[number % 100 / 10];
		ledBuffer->thirdReg = numbers[number % 1000 / 100];
		ledBuffer->fourthReg  = numbers[number / 1000];
	}
	if (number > 9999) {
		ledBuffer->firstReg  = numbers[9];
		ledBuffer->secondReg = numbers[9];
		ledBuffer->thirdReg = numbers[9];
		ledBuffer->fourthReg  = numbers[9];
	}



}

void getLedBufferFromNumberSpeed(uint32_t number, dysplayBufferStruct* ledBuffer) {

	ledBuffer->firstReg = numbers[0];
	ledBuffer->secondReg = numbers[0];
	ledBuffer->thirdReg = numbers[0];
	ledBuffer->fourthReg = numbers[0];

	if (number < 10) {
		ledBuffer->firstReg  = numbers[number];
		ledBuffer->secondReg = numbers[0];
		ledBuffer->thirdReg = numbers[0];
		ledBuffer->fourthReg  = numbers[0];
		ledBuffer->fourthReg=ledBuffer->fourthReg&~0b00010000;
	}
	if (number >= 10 && number < 100) {
		ledBuffer->firstReg  = numbers[number % 10];
		ledBuffer->secondReg = numbers[number / 10];
		ledBuffer->thirdReg = numbers[0];
		ledBuffer->fourthReg  = numbers[0];
		ledBuffer->fourthReg=ledBuffer->fourthReg&~0b00010000;
	}
	if (number >= 100 && number < 1000) {
		ledBuffer->firstReg  = numbers[number % 10];
		ledBuffer->secondReg = numbers[number % 100 / 10];
		ledBuffer->thirdReg = numbers[number / 100];
		ledBuffer->fourthReg  = numbers[0];
				ledBuffer->fourthReg=ledBuffer->fourthReg&~0b00010000;
	}
	if (number >= 1000 && number < 10000) {
		ledBuffer->firstReg  = numbers[number % 10];
		ledBuffer->secondReg = numbers[number % 100 / 10];
		ledBuffer->thirdReg = numbers[number % 1000 / 100];
		ledBuffer->fourthReg  = numbers[number / 1000];
		ledBuffer->fourthReg=ledBuffer->fourthReg&~0b00010000;
	}
	if (number >= 10000 && number < 100000) {

		ledBuffer->firstReg  = numbers[number % 100 / 10];
		ledBuffer->secondReg = numbers[number % 1000 / 100];
		ledBuffer->thirdReg = numbers[number % 10000 / 1000];
		ledBuffer->thirdReg=ledBuffer->thirdReg&~0b00010000;
		ledBuffer->fourthReg  = numbers[number / 10000];


	}
	if (number >= 100000 && number < 1000000) {

			ledBuffer->firstReg  = numbers[number % 1000 / 100];
			ledBuffer->secondReg = numbers[number % 10000 / 1000];
			ledBuffer->secondReg=ledBuffer->secondReg&~0b00010000;
			ledBuffer->thirdReg = numbers[number % 100000 / 10000];
			ledBuffer->fourthReg  = numbers[number / 100000];


		}


}

void getLedBufferFromNumberSpeedNotDot(uint32_t number, dysplayBufferStruct* ledBuffer) {

	ledBuffer->firstReg = numbers[0];
	ledBuffer->secondReg = numbers[0];
	ledBuffer->thirdReg = numbers[0];
	ledBuffer->fourthReg = numbers[0];

	if (number < 10) {
		ledBuffer->firstReg  = numbers[number];
		ledBuffer->secondReg = numbers[0];
		ledBuffer->thirdReg = numbers[0];
		ledBuffer->fourthReg  = numbers[0];
		ledBuffer->fourthReg=ledBuffer->fourthReg&~0b00010000;
	}
	if (number >= 10 && number < 100) {
		ledBuffer->firstReg  = numbers[number % 10];
		ledBuffer->secondReg = numbers[number / 10];
		ledBuffer->thirdReg = numbers[0];
		ledBuffer->fourthReg  = numbers[0];
		ledBuffer->fourthReg=ledBuffer->fourthReg&~0b00010000;
	}
	if (number >= 100 && number < 1000) {
		ledBuffer->firstReg  = numbers[number % 10];
		ledBuffer->secondReg = numbers[number % 100 / 10];
		ledBuffer->thirdReg = numbers[number / 100];
		ledBuffer->fourthReg  = numbers[0];
				ledBuffer->fourthReg=ledBuffer->fourthReg&~0b00010000;
	}
	if (number >= 1000 && number < 10000) {
		ledBuffer->firstReg  = numbers[number % 10];
		ledBuffer->secondReg = numbers[number % 100 / 10];
		ledBuffer->thirdReg = numbers[number % 1000 / 100];
		ledBuffer->fourthReg  = numbers[number / 1000];
		ledBuffer->fourthReg=ledBuffer->fourthReg&~0b00010000;
	}
	if (number >= 10000 && number < 100000) {

		ledBuffer->firstReg  = numbers[number % 100 / 10];
		ledBuffer->secondReg = numbers[number % 1000 / 100];
		ledBuffer->thirdReg = numbers[number % 10000 / 1000];
		ledBuffer->thirdReg=ledBuffer->thirdReg&~0b00010000;
		ledBuffer->fourthReg  = numbers[number / 10000];


	}
	if (number >= 100000 && number < 1000000) {

			ledBuffer->firstReg  = numbers[number % 1000 / 100];
			ledBuffer->secondReg = numbers[number % 10000 / 1000];
			ledBuffer->secondReg=ledBuffer->secondReg&~0b00010000;
			ledBuffer->thirdReg = numbers[number % 100000 / 10000];
			ledBuffer->fourthReg  = numbers[number / 100000];


		}


}


void getLedBufferFromNumberTime(uint32_t time, dysplayBufferStruct* ledBuffer) {
	uint32_t number = time/10;


	ledBuffer->firstReg = numbers[0];
	ledBuffer->secondReg = numbers[0];
	ledBuffer->thirdReg = numbers[0];
	ledBuffer->fourthReg = numbers[0];

	if (number < 10) {
		ledBuffer->firstReg  = numbers[number];
		ledBuffer->secondReg = numbers[0];
		ledBuffer->thirdReg = numbers[0];
		ledBuffer->fourthReg  = numbers[0];
		ledBuffer->thirdReg=ledBuffer->thirdReg&~0b00010000;
	}
	if (number >= 10 && number < 100) {
		ledBuffer->firstReg  = numbers[number % 10];
		ledBuffer->secondReg = numbers[number / 10];
		ledBuffer->thirdReg = numbers[0];
		ledBuffer->fourthReg  = numbers[0];
		ledBuffer->thirdReg=ledBuffer->thirdReg&~0b00010000;
	}
	if (number >= 100 && number < 1000) {
		ledBuffer->firstReg  = numbers[number % 10];
		ledBuffer->secondReg = numbers[number % 100 / 10];
		ledBuffer->thirdReg = numbers[number / 100];
		ledBuffer->fourthReg  = numbers[0];
		ledBuffer->thirdReg=ledBuffer->thirdReg&~0b00010000;
	}
	if (number >= 1000 && number < 10000) {
		ledBuffer->firstReg  = numbers[number % 10];
		ledBuffer->secondReg = numbers[number % 100 / 10];
		ledBuffer->thirdReg = numbers[number % 1000 / 100];
		ledBuffer->fourthReg  = numbers[number / 1000];
		ledBuffer->thirdReg=ledBuffer->thirdReg&~0b00010000;
	}
	if (number >= 10000 && number < 100000) {

		ledBuffer->firstReg  = numbers[number % 100 / 10];
		ledBuffer->secondReg = numbers[number % 1000 / 100];
		ledBuffer->thirdReg = numbers[number % 10000 / 1000];
		ledBuffer->secondReg=ledBuffer->secondReg&~0b00010000;
		ledBuffer->fourthReg  = numbers[number / 10000];


	}
	if (number >= 100000 && number < 1000000) {

			ledBuffer->firstReg  = numbers[number % 1000 / 100];
			ledBuffer->secondReg = numbers[number % 10000 / 1000];
			ledBuffer->firstReg=ledBuffer->firstReg&~0b00010000;
			ledBuffer->thirdReg = numbers[number % 100000 / 10000];
			ledBuffer->fourthReg  = numbers[number / 100000];


		}


}

