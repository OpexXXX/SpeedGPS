/*
 * ParameterMeter.h
 *
 *  Created on: 25 июн. 2018 г.
 *      Author: opex
 */

#ifndef PARAMETERMETER_H_
#define PARAMETERMETER_H_

typedef enum
{
	VEHICLE_STOPPED=0,
	VEHICLE_ACCELERATE,
	VEHICLE_BRAKES
}VehicleState;

typedef enum
{
	MES_BRAKES=0,
	MES_ACCELERATE,
	MES_STOPPED
}StatusOfMeasurement;

typedef enum
{
	MES_NOT_FIXED=0,
	MES_FIXED
}StatusOfIndMeasurement;


typedef struct  {
	uint32_t Time; //время
	uint32_t SLatitude;  //Латитуда
	char NS[3];                         //
	uint32_t SLongitude;         //Лонгитуда
	char EW[3];                         //
	uint32_t CourseTrue;                // курс
	uint32_t Speed; 	//скорость
	uint8_t StatusMeas;
} gpsSpeedInderStruct;   //Структура фиксации промежуточных результатов


uint32_t IntermediateMeasurementOfSpeed [] = {
		5000,
		10000,
		20000,
		30000,
		40000,
		50000,
		60000,
		70000,
		80000,
		90000,
		100000,
		110000,
		120000,
		130000,
		140000,
		150000,
		160000,
		170000,
		180000,
		190000,
		200000,
		210000,
		220000,
		230000,
		240000,
		250000,
		260000,
		270000,
		280000,
		290000,
};


#endif /* PARAMETERMETER_H_ */
