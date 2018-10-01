/*
 * gps.h
 *
 *  Created on: 21 июн. 2018 г.
 *      Author: opex
 */

#ifndef GPS_H_
#define GPS_H_

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

typedef enum {
	GPS_PRMC=1,
	GPS_NRMC,
	GPS_PGGA,
	GPS_NGGA,
	GPS_PGSV,
	GPS_LGSV,
	GPS_PVTG,
	GPS_NVTG,
}GPS_MESSEGE_TYPE;

typedef struct {
	uint32_t Time; //время
	uint32_t SLatitude;  //Латитуда
	char NS[3];                         //
	uint32_t SLongitude;         //Лонгитуда
	char EW[3];                         //
	uint32_t CourseTrue;                // курс
	uint32_t Speed; 	//скорость
} gpsSpeedMessegeStruct;



int AsciiToInt(char* s);
uint8_t Parser(unsigned char data);
uint32_t getDifTime(uint32_t startTime, uint32_t stopTime );
#endif /* GPS_H_ */
