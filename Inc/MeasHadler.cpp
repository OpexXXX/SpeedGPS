/*
 * MeasHadler.cpp
 *
 *  Created on: 10 окт. 2018 г.
 *      Author: opex
 */

#include "MeasHadler.h"
#include <math.h>
namespace Measurment {

MeasHadler::MeasHadler() {
	// TODO Автоматически созданная заглушка конструктора
}

MeasHadler::~MeasHadler() {
	// TODO !CodeTemplates.destructorstub.tododesc!
}

/* namespace Measurment */
/*проверка на прохождение расстояния*/
uint32_t MeasHadler::checkPassageDistance(uint16_t checkDistance) {
	/*Details:{Time = 205557900, SLatitude = 53.1645126, NS = "N\0", SLongitude = 92.2391434, altitude = 0, EW = "E\0", CourseTrue = 0, Speed = 766}
	 По умолчанию:{...}
	 De*/
	double curr_distance = getDistance(&measStrukt->StartMeas,
			&measStrukt->gpsData, 0);
	measStrukt->Distance = (curr_distance);
	double prev_distance = (getDistance(&measStrukt->StartMeas,
			&measStrukt->messageArray[3], 0));
	if (curr_distance > checkDistance && prev_distance < checkDistance) {

		return (1);
	} else {
		return (0);
	}
}
/*Получить дорасчетное время замера расстояния*/
uint32_t MeasHadler::getResultTimeForDistance(uint16_t checkDistance) {

	double curr_distance = getDistance(&measStrukt->StartMeas,
			&measStrukt->gpsData, 0);
	double prev_distance = getDistance(&measStrukt->StartMeas,
			&measStrukt->messageArray[3], 0);

	uint32_t resultTime = getDifTime(measStrukt->StartMeas.Time,
			measStrukt->messageArray[3].Time);
	float koef = 1 - ((curr_distance - 402) / (curr_distance - prev_distance));
	uint32_t difTime = (getDifTime(measStrukt->messageArray[3].Time,
			measStrukt->gpsData.Time)) * koef;

	resultTime += difTime;
	return (resultTime);
}

/*Получить растояние между точками*/

double Measurment::MeasHadler::getDistance(gpsMessege *firstCoor,
		gpsMessege *SecondCoor, bool altitud) {

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
/*Обработать пакет*/
void MeasHadler::processPackage(gpsMessege gpsData) {
	/*//Сдвигаем буфер с замерами*/
	for (uint8_t var = 0; var < 4; ++var) {
		measStrukt->messageArray[var] = measStrukt->messageArray[var + 1];
	}
	/*// Записываем текущий замер*/
	measStrukt->messageArray[4] = gpsData;
	measStrukt->gpsData = gpsData;

	/*// Берем среднюю скорость пяти замеров*/
	measStrukt->AvgSpeed = (measStrukt->messageArray[0].Speed
			+ measStrukt->messageArray[1].Speed
			+ measStrukt->messageArray[2].Speed
			+ measStrukt->messageArray[3].Speed
			+ measStrukt->messageArray[4].Speed) / 5;
	/*//Проверка на Остановку автомобиля, обнуление замеров, если 5 замеров отсутствует курс и скорость ниже 1 км/ч*/
	uint8_t flagRes = 0;
	/*//прогоняем все пять замеров*/
	for (int var = 0; var < 5; ++var) {
		flagRes += (measStrukt->messageArray[var].CourseTrue == 0); /*//проверяем отсутствие курса*/
		flagRes += (measStrukt->messageArray[var].Speed < 1000); /*// проверяем скорость ниже 1 км/ч*/
	}
	if (flagRes == 10) {
		measStrukt->VehicleStatus = VEHICLE_STOPPED;
	} /*//Если все пять замеров нулевые, считаем , что автомобиль остановлен и готов для старта*/
	if (measStrukt->VehicleStatus == VEHICLE_STOPPED) { /*// если автомобиль остановлен и готов для старта
	 //обнулить промежуточные итоги*/
		measStrukt->MeasurmentStatus = MES_STOPPED; /*// статус измерения "остановлен"*/
		measStrukt->Distance = 0;
	}
	/*
	 //Проверка на начала замера с места
	 //Если в текущем пакете ГПС появился курс, а в предыдущем он отсутствовал  и
	 //средняя скорость предыдущих двух замеров меньше 1 км/ч
	 */
	if (gpsData.CourseTrue != 0 && (measStrukt->messageArray[3].CourseTrue == 0)
			&& (((measStrukt->messageArray[3].Speed
					+ measStrukt->messageArray[2].Speed) / 2) < 1000)) {
		measStrukt->StartMeas = measStrukt->messageArray[3]; /*// Записываем точку старта*/
		measStrukt->VehicleStatus = VEHICLE_ACCELERATE; /*// статус автомобиля "Ускоряется"*/
		measStrukt->MeasurmentStatus = MES_ACCELERATE; /*// статус измерения "Ускоряется"*/
	}

}
/*проверяем переход через замеряемые величины скорости*/
uint32_t MeasHadler::checkSpeedThrough() {
	for (uint8_t i = 0; i < 29; ++i) {
		if (measStrukt->gpsData.Speed > IntermediateMeasurementOfSpeed[i]
				&& measStrukt->messageArray[3].Speed
						< IntermediateMeasurementOfSpeed[i]) {
			/*//TODO что то делаем*/
			return (IntermediateMeasurementOfSpeed[i]);
		}
	}
	return (0);
}
/*//Старт для замера торможения*/
uint32_t MeasHadler::checkForStartBreakMeas() {
	if (measStrukt->gpsData.Speed < 100000
			&& measStrukt->messageArray[3].Speed > 100000) {
		measStrukt->StartBreakMeas = measStrukt->messageArray[3];

		uint32_t startTime = getResultTimeForSpeed(100000);
		/*//TODO что то делаем*/
		return (startTime);
	}
	return 0;
}
/*//Получить дорасчетное время замера ускорения*/
uint32_t MeasHadler::getResultTimeForSpeed(uint16_t speed) {
	uint32_t resultTime = getDifTime(measStrukt->StartMeas.Time,
			measStrukt->messageArray[3].Time);
	float gpsDataSpeedF = measStrukt->gpsData.Speed;
	float messageArraySpeedF = measStrukt->messageArray[3].Speed;
	float koef = 1
			- ((gpsDataSpeedF - speed) / (gpsDataSpeedF - messageArraySpeedF));
	uint32_t difTime = (getDifTime(measStrukt->messageArray[3].Time,
			measStrukt->gpsData.Time)) * koef;
	resultTime += difTime;
	return (resultTime);
}
}
