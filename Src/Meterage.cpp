/*
 * AxelMeas.cpp
 *
 *  Created on: 10 окт. 2018 г.
 *      Author: opex
 */

#include "Meterage.h"

namespace Measurment {

Meterage::Meterage(gpsMessage startPoint) {
	// TODO Автоматически созданная заглушка конструктора


}

Meterage::~Meterage() {
	// TODO !CodeTemplates.destructorstub.tododesc!
}

    double getDistanceFromStart(bool altitud){} /*Получить растояние от стартовой точки*/

    uint32_t getResultTimeForSpeed(uint16_t speed){

    }/*//получить дорасчетное время замера ускорения*/
    uint32_t getResultTimeForDistance(uint16_t checkDistance) {

    }/*Получить дорасчетное время замера расстояния*/

    uint32_t checkSpeedThrough(bool altitud){} // Переход через измеряемую скорость
    uint32_t checkDistanceThrough(bool altitud){}/*проверяем переход через замеряемые величины расстояния*/

} /* namespace Measurment */
