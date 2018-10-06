/*
 * measurment.h
 *
 *  Created on: 5 окт. 2018 г.
 *      Author: opex
 */

#ifndef MEASURMENT_H_
#define MEASURMENT_H_


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
	double SLatitude;  //Латитуда
	uint32_t Altim;  //Высота
	char NS[3];                         //
	double SLongitude;         //Лонгитуда
	char EW[3];                         //
	uint32_t CourseTrue;                // курс
	uint32_t Speed; 	//скорость
	uint8_t StatusMeas;
} gpsSpeedInderStruct;   //Структура фиксации промежуточных результатов




	typedef struct  {
		uint8_t VehicleStatus;
			uint8_t MeasurmentStatus;
			portBASE_TYPE xStatus;
			gpsSpeedMessegeStruct messageArray[5];
			gpsSpeedMessegeStruct gpsData;
			gpsSpeedMessegeStruct StartMeas;
			gpsSpeedMessegeStruct StartBreakMeas;
			gpsSpeedInderStruct IntermediateRresults[30];
			uint32_t AvgSpeed;
	} measurmentStruct;   //Структура для работы с замером
	uint32_t checkSpeedThrough(measurmentStruct *this);

	uint32_t getResultTimeForSpeed(uint16_t speed, measurmentStruct *this) ;
	void processPackage(measurmentStruct *this,gpsSpeedMessegeStruct gpsData);

#endif /* MEASURMENT_H_ */
