/*
 * MeasHadler.cpp
 *
 *  Created on: 10 окт. 2018 г.
 *      Author: opex
 */

#include "MeasHadler.h"

namespace MeasHadler {


/* namespace Measurment */

/*Обработать пакет*/
void processPackage(gpsMessage gpsData) {
	/*//Сдвигаем буфер с замерами*/
	moveBuffer(gpsData);
	/*// Берем среднюю скорость пяти замеров*/
	getAvgSpeed();
	checkViecleStatus();
}

void checkViecleStatus() {
	/*//Проверка на Остановку автомобиля, обнуление замеров, если 5 замеров отсутствует курс и скорость ниже 1 км/ч*/
	uint8_t flagRes = 0;
	/*//прогоняем все пять замеров*/
	for (int var = 0; var < 5; ++var) {
		flagRes += (messageArray[var].CourseTrue == 0); /*//проверяем отсутствие курса*/
		flagRes += (messageArray[var].Speed < 1000); /*// проверяем скорость ниже 1 км/ч*/
	}
	if (flagRes == 10) {
		VehicleStatus = VEHICLE_STOPPED;
	} /*//Если все пять замеров нулевые, считаем , что автомобиль остановлен и готов для старта*/

	/*
	 //Проверка на начала замера с места
	 //Если в текущем пакете ГПС появился курс, а в предыдущем он отсутствовал  и
	 //средняя скорость предыдущих двух замеров меньше 1 км/ч
	 */

	if (gpsData.CourseTrue != 0 && (messageArray[3].CourseTrue == 0)
			&& (((messageArray[3].Speed + messageArray[2].Speed) / 2) < 1000)) {

		//StartMeas = measStrukt->messageArray[3]; /*// Записываем точку старта*/
		VehicleStatus = VEHICLE_ACCELERATE; /*// статус автомобиля "Ускоряется"*/
		MeasurmentStatus = MES_ACCELERATE; /*// статус измерения "Ускоряется"*/

	}
}

/*//Старт для замера торможения*/
uint32_t checkForStartBreakMeas() {
	if (gpsData.Speed < 100000 && messageArray[3].Speed > 100000) {
		//StartBreakMeas = messageArray[3];

		//uint32_t startTime = Met::getResultTimeForSpeed(100000);
		/*//TODO что то делаем*/
		//return (startTime);
	}
	return 0;
}

void getAvgSpeed() //Получить среднюю скорость
{
	uint32_t sumSpeed = 0;
	for (int i = 0; i < 5; ++i) {
		sumSpeed += messageArray[i].Speed;
	}
	AvgSpeed = sumSpeed / 5;
}

void moveBuffer(MeasHadler::gpsMessage gpsD) //Сдвиг буфера
		{
	for (uint8_t var = 0; var < 4; ++var) {
		messageArray[var] = messageArray[var + 1];
	}
	messageArray[4] = gpsD;
	gpsData = gpsD;
}

}
