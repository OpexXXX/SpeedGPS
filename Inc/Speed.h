/*
 * Accelerate.h
 *
 *  Created on: 11 окт. 2018 г.
 *      Author: opex
 */

#ifndef SPEED_H_
#define SPEED_H_

#include <Meterage.h>

namespace Measurment {

class Speed: public Meterage {
private:
    const uint32_t checkSpeed; //  Финишная скорость для замера
    uint32_t travelDistance; // Пройденное расстояние за замер

public:
	Speed();
	virtual ~Speed();
};

} /* namespace Measurment */

#endif /* SPEED_H_ */
