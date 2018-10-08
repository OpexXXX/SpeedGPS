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
	GPS_PRMC = 1,
	GPS_NRMC,
	GPS_PGGA,
	GPS_NGGA,
	GPS_PGSV,
	GPS_LGSV,
	GPS_PVTG,
	GPS_NVTG,
} GPS_MESSEGE_TYPE;

typedef struct {
	uint8_t Status;
	uint32_t Time; //время
	float SLatitude;  //Широта
	char NS[3];                         //Север/Юг
	float SLongitude;         //Долгота
	float altitude;
	char EW[3];                         //Запад/Восток
	uint32_t CourseTrue;                // курс
	uint32_t Speed; 	//скорость
} gpsSpeedMessegeStruct;

//Конвертация String в Int, строка без точки, "154" -> 154
int AsciiToInt(char* s);
//Конвертация String в Int, возвращает целую часть до точки "151.6654684" -> 154
uint32_t AsciiBeforeDotToInt(char* s);
//Конвертация String в Int, возвращает дробную часть после точки, "151.6654684" -> 6654684
uint32_t AsciiAfterDotToInt(char* s);
//Конвертация String в Int, возвращает число без точки, "151.6654684" -> 1516654684
uint32_t AsciiRemoveDotToInt(char* s);
//Парсер пакета UART
void uartParserGps(unsigned char data);
//Конвертация String в float, возвращает число , "151.6654684" -> 151.6654684
float stringToFloat(char *string);
//Возвращает разницу времени в секундах
uint32_t getDifTime(uint32_t startTime, uint32_t stopTime);
//Парсер посылки UART 1 байт
uint8_t Parser(unsigned char data);
//TODO Дописать функцию преобразования коо
float convertStrDegToDecimal(char*  coor);
#endif /* GPS_H_ */
