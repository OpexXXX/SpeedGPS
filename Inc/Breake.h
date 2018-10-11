/*
 * Breake.h
 *
 *  Created on: 11 окт. 2018 г.
 *      Author: opex
 */

#ifndef BREAKE_H_
#define BREAKE_H_

#include <Meterage.h>

namespace Measurment {

class Breake: public Meterage {
    private:
        const uint32_t startSpeed;
    uint32_t travelDistance;
public:
	Breake();
	virtual ~Breake();
};

} /* namespace Measurment */

#endif /* BREAKE_H_ */
