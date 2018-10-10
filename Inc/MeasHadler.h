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

//Структура для работы с замером
typedef struct {
	uint8_t VehicleStatus; //Состояние авто
	uint8_t MeasurmentStatus; // Статус замера
	gpsMessege messageArray[5]; // Массив текущих посылок для усреднений
	gpsMessege gpsData; //Текущая посылка
	gpsMessege StartMeas; //Посылка с места старта
	gpsMessege StartBreakMeas; // Посыска с места начала торможения
	gpsMessege IntermediateRresults[30]; // Массив для записи точек, нахер нужен?
	uint32_t AvgSpeed; //средняя скорость авто на 5 замеров
	uint32_t Distance; // Растояние до точки старта
} measurmentStruct;

class MeasHadler {
private:
	static uint8_t VehicleStatus; //Состояние авто
	static uint8_t MeasurmentStatus; // Статус замера
	static gpsMessege messageArray[5]; // Массив текущих посылок для усреднений
	static gpsMessege gpsData; //Текущая посылка
	static uint32_t AvgSpeed; //средняя скорость авто на 5 замеров


	uint32_t getResultTimeForSpeed(uint16_t speed);/*//получить дорасчетное время замера ускорения*/
	uint32_t getResultTimeForDistance(uint16_t checkDistance);/*Получить дорасчетное время замера расстояния*/




	/*//Старт для замера торможения*/
	static uint32_t checkForStartBreakMeas();
	/*проверка на старт с метса*/
	static uint32_t checkForStartViecle();
public:
	/*Обработать пакет от гпс модуля*/
	static void processPackage(gpsMessege gpsData) ;
	//void Measurment::MeasHadler::processPackage(gpsMessege gpsData);
	MeasHadler();
	virtual ~MeasHadler();
};

} /* namespace Measurment */

#endif /* MEASHADLER_H_ */
