/*
 * AxelMeas.cpp
 *
 *  Created on: 10 окт. 2018 г.
 *      Author: opex
 */

#include "Meterage.h"

namespace Met {

Meterage::Meterage(MeasHadler::gpsMessage startPoint) {
	// TODO Автоматически созданная заглушка конструктора

}

Meterage::~Meterage() {
	// TODO !CodeTemplates.destructorstub.tododesc!
}

double Meterage::getDistanceFromStart(bool altitud) {
} /*Получить растояние от стартовой точки*/



uint32_t Speed::checkSpeedThrough(bool altitud) {
} // Переход через измеряемую скорость
uint32_t Distance::checkDistanceThrough(bool altitud) {
}/*проверяем переход через замеряемые величины расстояния*/

/*Получить дорасчетное время замера расстояния*/
uint32_t Meterage::getResultTimeForDistance(uint16_t checkDistance) {

	double curr_distance = getDistance(&startPoint,&(MeasHadler::gpsData), false);
	double prev_distance = getDistance(&startPoint,&MeasHadler::messageArray[3], 0);

	uint32_t resultTime = getDifTime(measStrukt->StartMeas.Time,
			measStrukt->messageArray[3].Time);
	float koef = 1 - ((curr_distance - 402) / (curr_distance - prev_distance));
	uint32_t difTime = (getDifTime(measStrukt->messageArray[3].Time,
			measStrukt->gpsData.Time)) * koef;

	resultTime += difTime;
	return (resultTime);
}

/*Получить растояние между точками*/

double getDistance(MeasHadler::gpsMessage *firstCoor,
		MeasHadler::gpsMessage *SecondCoor, bool altitud) {
	/*//Radians = (dd.ff)*pi/180*/
	double lat_1 = firstCoor->SLatitude * (M_PI / 180);
	double lat_2 = SecondCoor->SLatitude * (M_PI / 180);
	double d_lon = SecondCoor->SLongitude * (M_PI / 180)
			- firstCoor->SLongitude * (M_PI / 180);
	double d_alt = firstCoor->altitude - SecondCoor->altitude;
	double angl = atan(
			(sqrt(
					pow(cos(lat_2) * sin(d_lon), 2)
							+ pow(
									cos(lat_1) * sin(lat_2)
											- sin(lat_1) * cos(lat_2)
													* cos(d_lon), 2))
					/ (sin(lat_1) * sin(lat_2)
							+ cos(lat_1) * cos(lat_2) * cos(d_lon))));
	double dist_2 = angl * 6371000;
	if (altitud) {
		double dist_alt = sqrt(pow(dist_2, 2) + pow(d_alt, 2));
		return (dist_alt);
	} else {
		return (dist_2);
	}
}

/*проверка на прохождение расстояния*/
bool Distance::checkPassageDistance(bool altitud=false) {
	/*Details:{Time = 205557900, SLatitude = 53.1645126, NS = "N\0", SLongitude = 92.2391434, altitude = 0, EW = "E\0", CourseTrue = 0, Speed = 766}
	 По умолчанию:{...}
	 De*/
	double curr_distance = getDistance(&startPoint,&(MeasHadler::gpsData), altitud);
	double prev_distance = getDistance(&startPoint,&(MeasHadler::messageArray[3]), altitud);

	if (curr_distance > checkDist && prev_distance < checkDist) {
		return (1);
	} else {
		return (0);
	}
}

/*//Получить дорасчетное время замера ускорения*/
uint32_t Speed::getResultTimeForSpeed(uint16_t speed) {
	uint32_t resultTime = getDifTime(measStrukt->StartMeas.Time,measStrukt->messageArray[3].Time);
	float gpsDataSpeedF = measStrukt->gpsData.Speed;
	float messageArraySpeedF = measStrukt->messageArray[3].Speed;
	float koef = 1
			- ((gpsDataSpeedF - speed) / (gpsDataSpeedF - messageArraySpeedF));
	uint32_t difTime = (getDifTime(measStrukt->messageArray[3].Time,
			measStrukt->gpsData.Time)) * koef;
	resultTime += difTime;
	return (resultTime);
}

} /* namespace Measurment */
