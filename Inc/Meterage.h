/*
 * AxelMeas.h
 *
 *  Created on: 10 окт. 2018 г.
 *      Author: opex
 */

#ifndef ACCELERATION_H_
#define ACCELERATION_H_

#include "MeasHadler.h"
#include <math.h>
namespace Met {

double getDistance(MeasHadler::gpsMessage *firstCoor,//Получить расстояние между точками
		MeasHadler::gpsMessage *SecondCoor, bool altitud);

class Meterage {
	MeasHadler::gpsMessage startPoint; //Посылка с места старта
	MeasHadler::gpsMessage finishPoint; //Посылка с финиша

	uint32_t resultTime; // Итоговое время замера
	double getDistanceFromStart(bool altitud); /*Получить растояние от стартовой точки*/




	/*Получить дорасчетное время замера расстояния*/
	uint32_t checkPassageDistance(uint16_t checkDistance);
	/*//Получить дорасчетное время замера ускорения*/
public:

			MeasHadler::gpsMessage *SecondCoor, bool altitud);
	virtual void chekMeasurment();
	Meterage(MeasHadler::gpsMessage startPoint);
	virtual ~Meterage();
};

class Breake: public Meterage {
private:
	const uint32_t startSpeed;
	uint32_t travelDistance; //Пройденное расстояние
public:
	Breake();
	virtual ~Breake();
};

class Distance: public Meterage {
private:

	const uint32_t checkDist; // Замеряемая дистанция
	uint32_t distanceExitSpeed; // Скорость на выходе из дистанции
	uint32_t getResultTimeForDistance(uint16_t checkDistance);/*Получить дорасчетное время замера расстояния*/
public:
	bool checkPassageDistance(bool altitud);/*проверяем переход через замеряемые величины расстояния*/
	Distance();
	virtual ~Distance();
};

class Speed: public Meterage {
private:
	const uint32_t checkSpeed; //  Финишная скорость для замера
	uint32_t travelDistance; // Пройденное расстояние за замер
	uint32_t getResultTimeForSpeed(uint16_t speed);/*//получить дорасчетное время замера ускорения*/
public:
	uint32_t checkSpeedThrough(bool altitud); // Переход через измеряемую скорость
	Speed();
	virtual ~Speed();
};

} /* namespace Measurment */

#endif /* ACCELERATION_H_ */
