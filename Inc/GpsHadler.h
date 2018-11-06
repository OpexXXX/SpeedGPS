/*
 * GpsHadler.h
 *
 *  Created on: 10 окт. 2018 г.
 *      Author: opex
 */

#ifndef GPSHADLER_H_
#define GPSHADLER_H_
#include <stdint.h>
namespace Gps {

typedef enum {
	GPS_NULL = 0,
	GPS_PRMC = 1,
	GPS_NRMC,
	GPS_PGGA,
	GPS_NGGA,
	GPS_PGSV,
	GPS_LGSV,
	GPS_PVTG,
	GPS_NVTG,
	GPS_GPGSA
} GPS_MESSEGE_TYPE;

typedef struct {
	uint8_t Status;
	uint32_t Time; //время
	float SLatitude;  //Широта
	float SLongitude;         //Долгота
	float altitude;	//Высота
	uint32_t CourseTrue;                // курс
	uint32_t Speed; 	//скорость
} gpsMessege;

//Конвертация String в Int, строка без точки, "154" -> 154
int asciiToInt(char* s);

//Конвертация String в Int, возвращает целую часть до точки "151.6654684" -> 154
uint32_t asciiBeforeDotToInt(char* s);

//Конвертация String в Int, возвращает дробную часть после точки, "151.6654684" -> 6654684
uint32_t asciiAfterDotToInt(char* s);

//Конвертация String в Int, возвращает число без точки, "151.6654684" -> 1516654684
uint32_t asciiRemoveDotToInt(char* s);

//Конвертация String в float, возвращает число , "151.6654684" -> 151.6654684
float stringToFloat(char *string);

//Возвращает разницу времени в секундах
uint32_t getDifTime(uint32_t startTime, uint32_t stopTime);

//TODO Дописать функцию преобразования коо
float convertStrDegToDecimal(char *coor);

class GpsHadler {
public:
	GpsHadler();
	gpsMessege getMessege();               //Подготовить посылку
	GPS_MESSEGE_TYPE charParser(unsigned char);		//Парсер пакета UART

	virtual
	~GpsHadler();
private:
	char AltitudaMSL[12]; /*//высота*/
		char SLatitude[16]; /*//Широта*/
		char SLongitude[12]; /*//Долгота*/
		char Speed[8]; /*//скорость*/
		char SatCount[4]; /*//используемых спутников*/
		char Data[12]; /*//Дата*/
		char Time[12]; /*//время*/
		char ViewSat[4];
		char COGstat[4];
		char Status[2]; /*//валидность*/
		char COG[8];
		char Knot[8];


	char NS[3];

	char EW[3];
	char CourseTrue[10]; /*// курс*/







	char SpeedAlt[8];
	char UNUSED[32]; /*//мусорка, тут все данные, которые не нужны*/

	unsigned char GLONAS_COUNT;
	unsigned char GPS_COUNT;
	volatile char DataDone;
	unsigned char DataValid;
	char * const RMC[14];
	char * const GGA[16];
	char * const GSV[21];
	char * const VTG[10];
	char * const GSA[18];
};

} /* namespace Gps */

#endif /* GPSHADLER_H_ */
