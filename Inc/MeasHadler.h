/*
 * MeasHadler.h
 *
 *  Created on: 10 окт. 2018 г.
 *      Author: opex
 */
#pragma once
#ifndef MEASHADLER_H_
#define MEASHADLER_H_

#include <stdint.h>
#include <math.h>

namespace Meas {

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

class Hadler {
public:

	Hadler();
	virtual ~Hadler();
	static VehicleState VehicleStatus; //Состояние авто
	static StatusOfMeasurement MeasurmentStatus; // Статус замера
	static uint32_t AvgSpeed; //средняя скорость авто на 5 замеров


	/*//Старт для замера торможения*/
	static uint32_t checkForStartBreakMeas();
	/*проверка на старт с места*/
	static uint32_t checkForStartMeas();
	/*проверка на старт с места*/
	static uint32_t checkForStartViecle();
	/*проверка на остановленное состояние*/
	static uint32_t checkForViecleStop();

	static void getAvgSpeed(); // Получить среднюю скорость
	static gpsMessage getCurrentGPS(); // Получить текущую посылку ГПС
	static gpsMessage getPrevousGPS(); // Получить предыдущую посылку ГПС

	static void Buffer(gpsMessage gpsD); //Сдвинуть буфер
	static StatusOfMeasurement checkMeasurmentStatus();
	static void checkViecleStatus();
	/*Обработать пакет от гпс модуля*/
	static void processPackage(gpsMessage gpsD);

private:
	static gpsMessage messageArray[5]; // Буффер текущих посылок для усреднений
		static gpsMessage gpsData; //Текущая посылка
};

} /* namespace Measurment */

#endif /* MEASHADLER_H_ */
