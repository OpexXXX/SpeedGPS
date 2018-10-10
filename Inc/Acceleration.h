/*
 * AxelMeas.h
 *
 *  Created on: 10 окт. 2018 г.
 *      Author: opex
 */

#ifndef ACCELERATION_H_
#define ACCELERATION_H_

#include <MeasHadler.h>

namespace Measurment {

class Acceleration: public MeasHadler {
	gpsMessege startPoint; //Посылка с места старта
	uint32_t distance; // Растояние до точки старта
public:
	/*проверяем переход через замеряемые величины скорости*/
		uint32_t checkSpeedThrough();
		uint32_t checkDistanceThrough(bool altitud);/*Получить растояние от стартовой точки*/

	Acceleration(gpsMessege startPoint);
	virtual ~Acceleration();
};

} /* namespace Measurment */

#endif /* ACCELERATION_H_ */
