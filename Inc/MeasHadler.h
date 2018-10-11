/*
 * MeasHadler.h
 *
 *  Created on: 10 окт. 2018 г.
 *      Author: opex
 */

#ifndef MEASHADLER_H_
#define MEASHADLER_H_
#include <stdint.h>
namespace Measurment {

typedef struct {
	uint8_t Status;
	uint32_t Time; //время
	float SLatitude;  //Широта
	float SLongitude;         //Долгота
	float altitude;	//Высота
	uint32_t CourseTrue;                // курс
	uint32_t Speed; 	//скорость
} gpsMessege;

typedef enum {
	VEHICLE_STOPPED = 1,
	VEHICLE_ACCELERATE,
	VEHICLE_BRAKES
} VehicleState;

typedef enum {
	MES_BRAKES = 1,
	MES_ACCELERATE,
	MES_STOPPED
} StatusOfMeasurement;

typedef enum {
	MES_NOT_FIXED = 1, MES_FIXED
} StatusOfIndMeasurement;

class MeasHadler {
private:

	static VehicleState VehicleStatus; //Состояние авто
	static StatusOfMeasurement MeasurmentStatus; // Статус замера
	static gpsMessege messageArray[5]; // Буффер текущих посылок для усреднений
	static gpsMessege gpsData; //Текущая посылка
	static uint32_t AvgSpeed; //средняя скорость авто на 5 замеров

	/*//Старт для замера торможения*/
	static uint32_t checkForStartBreakMeas();
	/*проверка на старт с метса*/
	static uint32_t checkForStartMeas();
	/*проверка на старт с метса*/
	static uint32_t checkForStartViecle();
	/*проверка на остановленное состояние*/
	static uint32_t checkForViecleStop();

	static uint32_t getAvgSpeed(); // Получить среднюю скорость
	static void moveBuffer(gpsMessege gpsData); //Сдвинуть буфер

	static double getDistance(gpsMessege *firstCoor,	//Получить расстояние между точками
		gpsMessege *SecondCoor, bool altitud)
public:
	/*Обработать пакет от гпс модуля*/
	static void processPackage(gpsMessege gpsData) ;
	//void Measurment::MeasHadler::processPackage(gpsMessege gpsData);
	MeasHadler();
	virtual ~MeasHadler();
};

} /* namespace Measurment */

#endif /* MEASHADLER_H_ */
