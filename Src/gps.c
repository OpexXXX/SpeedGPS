/*
 * gps.c
 *
 *  Created on: 21 июн. 2018 г.
 *      Author: opex
 */
#include "gps.h"
#include "ledDriver.h"
#include <stdio.h>
#include <string.h>
char Time[12] = ""; //время
char Status[2] = ""; //валидность
char SLatitude[16] = "";  //Широта
char NS[3] = "";                          //
char SLongitude[12] = "";         //Долгота
char EW[3] = "";                          //
char CourseTrue[10] = "";                 // курс
char Data[12] = "";                               //Дата
char SatCount[4] = "";                    //используемых спутников
char AltitudaMSL[12] = "";            //высота
char ViewSat[4];   //
char COG[8] = "";                 //
char COGstat[4] = "";             //
char Speed[8] = "";                       //скорость
char SpeedAlt[8] = "";    //
char UNUSED[32] = "";                //мусорка, тут все данные, которые не нужны
char Knot[8] = "";
char * const RMC[] = { Time, Status, SLatitude, NS, SLongitude, EW, Speed,
		CourseTrue, Data, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED };
char * const GGA[] = { UNUSED, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED, SatCount,
		UNUSED, AltitudaMSL, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED,
		UNUSED };
char * const GSV[] = { UNUSED, UNUSED, ViewSat, UNUSED, UNUSED, UNUSED, UNUSED,
		UNUSED, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED,
		UNUSED, UNUSED, UNUSED, UNUSED, UNUSED };
char * const VTG[] = { COG, COGstat, UNUSED, UNUSED, Knot, UNUSED, UNUSED,
		UNUSED, UNUSED, UNUSED };
unsigned char GLONAS_COUNT = 0;
unsigned char GPS_COUNT = 0;
volatile char DataDone = 0;
unsigned char DataValid = 0;

extern osMessageQId GPSHandlerHandle;

/*
 /* Converting Between Decimal Degrees, Degrees, Minutes and Seconds,
 * and Radians (dd + mm/60 +ss/3600) to Decimal degrees (dd.ff)
 *
 * dd = whole degrees, mm = minutes, ss = seconds
 *
 * dd.ff = dd + mm/60 + ss/3600
 *
 * Example: 30 degrees 15 minutes 22 seconds = 30 + 15/60 +
 * 22/3600 = 30.2561
 *
 * Decimal degrees (dd.ff) to (dd + mm/60 +ss/3600)
 *
 * For the reverse conversion, we want to convert dd.ff to dd mm
 * ss. Here ff = the fractional part of a decimal degree.
 *
 * mm = 60*ff
 *
 * ss = 60*(fractional part of mm)
 *
 * Use only the whole number part of mm in the final result.
 *
 * 30.2561 degrees = 30 degrees
 *
 * .2561*60 = 15.366 minutes
 *
 * .366 minutes = 22 seconds, so the final result is 30 degrees 15
 * minutes 22 seconds
 *
 * Decimal degrees (dd.ff) to Radians
 *
 * Radians = (dd.ff)*pi/180
 *
 * Radians to Decimal degrees (dd.ff)
 *
 * (dd.ff) = Radians*180/pi
 */
//Конвертация String в Int, строка без точки, "154" -> 154
int AsciiToInt(char* s) {
	int n = 0;
	while (*s >= '0' && *s <= '9') {
		n *= 10;
		n += *s++;
		n -= '0';
	}
	return n;
}
//Конвертация String в Int, возвращает целую часть до точки "151.6654684" -> 154
uint32_t AsciiBeforeDotToInt(char* s) {
	uint32_t n = 0;
	while (*s >= '0' && *s <= '9') {
		if (*s == '.')
			return n;
		n *= 10;
		n += *s++;
		n -= '0';
	}
	return n;
}
//Конвертация String в Int, возвращает дробную часть после точки, "151.6654684" -> 6654684
uint32_t AsciiAfterDotToInt(char* s) {
	uint32_t n = 0;
	uint8_t flagDot = 0;
	while (*s) {
		if (*s == '.') {
			flagDot = 1;
			s++;
		}
		if (flagDot && (*s)) {
			n *= 10;
			n += *s++;
			n -= '0';
		} else {
			s++;
		}

	}
	return n;
}
//Конвертация String в Int, возвращает число без точки, "151.6654684" -> 1516654684
uint32_t AsciiRemoveDotToInt(char* s)

{
	uint32_t n = 0;
	while (*s) {
		if (*s == '.') {

			s++;
		}
		if (*s) {
			n *= 10;
			n += *s++;
			n -= '0';
		} else {
			s++;
		}

	}
	return n;
}
//Парсер пакета UART
void uartParserGps(unsigned char data) {

	portBASE_TYPE xStatus;

	switch (Parser(data)) {
	case GPS_NRMC:
		if (Status[0] != 'K') {
			gpsSpeedMessegeStruct gpsUInt;
			gpsUInt.Status = Status[0];
			gpsUInt.Time = AsciiRemoveDotToInt(&Time) * 10;
			float tempSpeed = AsciiRemoveDotToInt(&Speed);
			tempSpeed = tempSpeed * 1.852;
			gpsUInt.Speed = tempSpeed;
			memcpy(gpsUInt.NS, NS, sizeof(NS));
			gpsUInt.SLatitude = convertStrDegToDecimal(&SLatitude);
			memcpy(gpsUInt.EW, EW, sizeof(EW));
			gpsUInt.SLongitude = convertStrDegToDecimal(&SLongitude);
			gpsUInt.CourseTrue = stringToFloat(&CourseTrue);
			xStatus = xQueueSendToBack(GPSHandlerHandle, &gpsUInt, 0);

	}
		break;
	default:
		break;
	}

}

//Конвертация String в float, возвращает число , "151.6654684" -> 151.6654684
float stringToFloat(char *string) {
	float result = 0.0;
	int len = strlen(string);
	int dotPosition = 0;

	for (int i = 0; i < len; i++) {
		if (string[i] == '.') {
			dotPosition = len - i - 1;
		} else {
			result = result * 10.0 + (string[i] - '0');
		}
	}

	while (dotPosition--) {
		result /= 10.0;
	}

	return result;
}
//Возвращает разницу времени в секундах
uint32_t getDifTime(uint32_t startTime, uint32_t stopTime) {
	uint64_t result = 0;
	int64_t hour = 0;
	int64_t minute = 0;
	int64_t seconds = 0;
//150012300
	int64_t difHour = (stopTime / 10000000) - (startTime / 10000000);

	int64_t difMinute = (stopTime / 100000) % 100 + difHour * 60
			- (startTime / 100000) % 100;

	int64_t difSecons = (stopTime % 100000) + difMinute * 60000
			- (startTime % 100000);

	result = difSecons;
	return result;

}
//Парсер посылки UART 1 байт
uint8_t Parser(unsigned char data) {
	static unsigned char ByteCount = 0xff;
	static unsigned int MsgType;
	static char *MsgTxt = (char*) &MsgType;
	static unsigned char ComaPoint = 0xff;
	static unsigned char CharPoint = 0;
	if (data == '$') {
		ByteCount = 0;
		ComaPoint = 0xff;
		MsgTxt = (char*) &MsgType;
		return 0;
	} //ждем начала стрки
	if (ByteCount == 0xff)
		return 0;                                                             //
	ByteCount++;
	if (ByteCount <= 1)
		return 0;                                                         //
	if (ByteCount < 6 && ByteCount > 1)            //берем 4 символа заголовка
			{
		*MsgTxt = data;   //и делаем из него число
		MsgTxt++;
		return 0;
	}
	//
	switch (MsgType) {
	case 0x434D5250:                             //GPRMC
	case 0x434D524E:                             //GNRMC
		if (data == ',') {
			ComaPoint++;
			CharPoint = 0;
			RMC[ComaPoint][0] = 0;
			return 0;
		}
		if ((data) == ('*')) {
			MsgType = 0;

			return GPS_NRMC;
		}
		RMC[ComaPoint][CharPoint++] = data;
		RMC[ComaPoint][CharPoint] = 0;
		return 0;
	case 0x41474750:                             //PGGA
	case 0x4147474e:                             //NGGA
		if (data == ',') {
			ComaPoint++;
			CharPoint = 0;
			GGA[ComaPoint][0] = 0;
			return 0;
		}
		if (data == '*') {
			MsgType = 0;
			return GPS_NGGA;
		}
		GGA[ComaPoint][CharPoint++] = data;
		GGA[ComaPoint][CharPoint] = 0;
		return 0;
	case 0x47545650:             //PVTG
		if (data == ',') {
			ComaPoint++;
			CharPoint = 0;
			VTG[ComaPoint][0] = 0;
			return 0;
		}
		if (data == '*') {
			return GPS_PVTG;
		}
		VTG[ComaPoint][CharPoint++] = data;
		VTG[ComaPoint][CharPoint] = 0;
		return 0;
	case 0x4754564e:             //NVTG
		if (data == ',') {
			ComaPoint++;
			CharPoint = 0;
			VTG[ComaPoint][0] = 0;
			return 0;
		}
		if (data == '*') {
			return GPS_NVTG;
		}
		VTG[ComaPoint][CharPoint++] = data;
		VTG[ComaPoint][CharPoint] = 0;
		return 0;
	case 0x56534750:             //PGSV
		if (data == ',') {
			ComaPoint++;
			CharPoint = 0;
			GSV[ComaPoint][0] = 0;
			return 0;
		}
		if (data == '*') {
			GPS_COUNT = AsciiToInt(ViewSat);
			MsgType = 0;
			return GPS_PGSV;
		}
		GSV[ComaPoint][CharPoint++] = data;
		GSV[ComaPoint][CharPoint] = 0;
		return 0;
	case 0x5653474c:             //LGSV
		if (data == ',') {
			ComaPoint++;
			CharPoint = 0;
			GSV[ComaPoint][0] = 0;
			return 0;
		}
		if (data == '*') {
			GLONAS_COUNT = AsciiToInt(ViewSat);
			MsgType = 0;
			return GPS_LGSV;
		}
		GSV[ComaPoint][CharPoint++] = data;
		GSV[ComaPoint][CharPoint] = 0;
		return 0;
	default:
		ByteCount = 0xff;
		break;
	}
	ByteCount = 0xff;
	return 0;
}
//TODO Дописать функцию преобразования координат в десятичный вид
float convertStrDegToDecimal(char*  coor) {
	// координаты приходят в виде "09224.91216" где 92 градуса 24.91216 минуты, секунд в посылке нет!!!
	float resultDecCoor, ss;
	uint16_t dd, mm;
	float fCoor = stringToFloat(coor) / 100;
	dd = fCoor;
	mm = ((fCoor - dd) * 100);
	ss = ((fCoor - dd) * 100) - mm;

	resultDecCoor = dd + ((float) mm / 60) + ((float) ss / 60);
	return resultDecCoor;
}
