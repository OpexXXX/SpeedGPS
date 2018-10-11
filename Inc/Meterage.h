/*
 * AxelMeas.h
 *
 *  Created on: 10 окт. 2018 г.
 *      Author: opex
 */

#ifndef ACCELERATION_H_
#define ACCELERATION_H_

#include "MeeasHadler.h"

namespace Measurment {

class Meterage: public MeasHadler {
	static gpsMessege startPoint; //Посылка с места старта
	gpsMessege finishPoint; //Посылка с места старта
    uint32_t resultTime; // Итоговое время замера

	double getDistanceFromStart(bool altitud); /*Получить растояние от стартовой точки*/

	uint32_t getResultTimeForSpeed(uint16_t speed);/*//получить дорасчетное время замера ускорения*/
	uint32_t getResultTimeForDistance(uint16_t checkDistance);/*Получить дорасчетное время замера расстояния*/

    uint32_t checkSpeedThrough(bool altitud); // Переход через измеряемую скорость
	uint32_t checkDistanceThrough(bool altitud);/*проверяем переход через замеряемые величины расстояния*/

public:
	Meterage(gpsMessege startPoint);
	virtual ~Meterage();
};

} /* namespace Measurment */

#endif /* ACCELERATION_H_ */
