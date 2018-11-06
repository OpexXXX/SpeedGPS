/*
 * MeasHadler.h
 *
 *  Created on: 10 окт. 2018 г.
 *      Author: opex
 */

#ifndef MEASHADLER_H_
#define MEASHADLER_H_
#include <stdint.h>
#include <math.h>


namespace MeasHadler {
typedef struct {
	uint8_t Status;
	uint32_t Time; //время
	float SLatitude;  //Широта
	float SLongitude;         //Долгота
	float altitude;	//Высота
	uint32_t CourseTrue;                // курс
	uint32_t Speed; 	//скорость
} gpsMessage;
typedef enum {
	VEHICLE_STOPPED = 1, VEHICLE_ACCELERATE, VEHICLE_BRAKES
} VehicleState;
typedef enum {
	MES_BRAKES = 1, MES_ACCELERATE, MES_STOPPED
} StatusOfMeasurement;
typedef enum {
	MES_NOT_FIXED = 1, MES_FIXED
} StatusOfIndMeasurement;



	 VehicleState VehicleStatus; //Состояние авто
	 StatusOfMeasurement MeasurmentStatus; // Статус замера


	uint32_t AvgSpeed; //средняя скорость авто на 5 замеров
	/*//Старт для замера торможения*/
	uint32_t checkForStartBreakMeas();
	/*проверка на старт с места*/
	uint32_t checkForStartMeas();
	/*проверка на старт с места*/
	uint32_t checkForStartViecle();
	/*проверка на остановленное состояние*/
	 uint32_t checkForViecleStop();
	 void getAvgSpeed(); // Получить среднюю скорость
	 void moveBuffer(gpsMessage gpsData); //Сдвинуть буфер
	 gpsMessage messageArray[5]; // Буффер текущих посылок для усреднений
	 gpsMessage gpsData; //Текущая посылка
	 StatusOfMeasurement checkMeasurmentStatus();
	 void checkViecleStatus();
	/*Обработать пакет от гпс модуля*/
	 void processPackage(gpsMessage gpsData);

} /* namespace Measurment */

#endif /* MEASHADLER_H_ */
