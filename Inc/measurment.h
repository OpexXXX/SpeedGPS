/*
 * measurment.h
 *
 *  Created on: 5 окт. 2018 г.
 *      Author: opex
 */

#ifndef MEASURMENT_H_
#define MEASURMENT_H_

typedef enum {
	VEHICLE_STOPPED = 0, VEHICLE_ACCELERATE, VEHICLE_BRAKES
} VehicleState;
typedef enum {
	MES_BRAKES = 0, MES_ACCELERATE, MES_STOPPED
} StatusOfMeasurement;
typedef enum {
	MES_NOT_FIXED = 0, MES_FIXED
} StatusOfIndMeasurement;
//Структура фиксации промежуточных результатов
typedef struct {
	uint32_t Time; //время
	float SLatitude;  //Широта
	uint32_t Altim;  //Высота
	char NS[3];                         //
	float SLongitude;         //Долгота
	char EW[3];                         //
	uint32_t CourseTrue;                // курс
	uint32_t Speed; 	//скорость
	uint8_t StatusMeas;
} gpsSpeedInderStruct;
//Структура для работы с замером
typedef struct {
	uint8_t VehicleStatus; //Состояние авто
	uint8_t MeasurmentStatus; // Статус замера
	gpsSpeedMessegeStruct messageArray[5]; // Массив текущих посылок для усреднений
	gpsSpeedMessegeStruct gpsData; //Текущая посылка
	gpsSpeedMessegeStruct StartMeas; //Посылка с места старта
	gpsSpeedMessegeStruct StartBreakMeas; // Посыска с места начала торможения
	gpsSpeedInderStruct IntermediateRresults[30]; // Массив для записи точек, нахер нужен?
	uint32_t AvgSpeed; //средняя скорость авто на 5 замеров
} measurmentStruct;

//проверяем переход через замеряемые величины скорости на ускорении
uint32_t checkSpeedThrough(measurmentStruct *this);
//Старт для замера торможения
uint32_t checkForStartBreakMeas(measurmentStruct *this)
//Получить дорасчетное время замера ускорения
uint32_t getResultTimeForSpeed(uint16_t speed, measurmentStruct *this);
//Обработчик пакетов
void processPackage(measurmentStruct *this, gpsSpeedMessegeStruct gpsData);
//Получить растояние между точками
double getDistance(gpsSpeedMessegeStruct *firstCoor,
		gpsSpeedMessegeStruct *SecondCoor);
#endif /* MEASURMENT_H_ */
