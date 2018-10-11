/*
 * Distance.h
 *
 *  Created on: 11 окт. 2018 г.
 *      Author: opex
 */

#ifndef DISTANCE_H_
#define DISTANCE_H_

#include <Meterage.h>

namespace Measurment {

class Distance: public Meterage {
    private:
    const uint32_t checkDist; // Замеряемая дистанция
    uint32_t distanceExitSpeed; // Скорость на выходе из дистанции
public:
	Distance();
	virtual ~Distance();
};

} /* namespace Measurment */

#endif /* DISTANCE_H_ */
