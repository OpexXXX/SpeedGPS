/*
 * measurmet.c
 *
 *  Created on: 5 окт. 2018 г.
 *      Author: opex
 */
#include "gps.h"
#include "measurment.h"

uint32_t IntermediateMeasurementOfSpeed [] = {
		5000,
		10000,
		20000,
		30000,
		40000,
		50000,
		60000,
		70000,
		80000,
		90000,
		100000,
		110000,
		120000,
		130000,
		140000,
		150000,
		160000,
		170000,
		180000,
		190000,
		200000,
		210000,
		220000,
		230000,
		240000,
		250000,
		260000,
		270000,
		280000,
		290000,
};


void processPackage(measurmentStruct *this, gpsSpeedMessegeStruct gpsData) {
	//Сдвигаем буфер с замерами
	for (uint8_t var = 0; var < 4; ++var) {
		this->messageArray[var] = this->messageArray[var + 1];
	}
	// Записываем текущий замер
	this->messageArray[4] = gpsData;

	this->gpsData = gpsData;
	// Берем среднюю скорость пяти замеров
	this->AvgSpeed = (this->messageArray[0].Speed + this->messageArray[1].Speed
			+ this->messageArray[2].Speed + this->messageArray[3].Speed
			+ this->messageArray[4].Speed) / 5;

	//Проверка на Остановку автомобиля, обнуление замеров, если 5 замеров отсутствует курс и скорость ниже 1 км/ч
	uint8_t flagRes = 0;
	//прогоняем все пять замеров
	for (int var = 0; var < 5; ++var) {
		flagRes += (this->messageArray[var].CourseTrue == 0); //проверяем отсутствие курса
		flagRes += (this->messageArray[var].Speed < 1000); // проверяем скорость ниже 1 км/ч
	}

	if (flagRes == 10) {
		this->VehicleStatus = VEHICLE_STOPPED;
	} //Если все пять замеров нулевые, считаем , что автомобиль остановлен и готов для старта

	if (this->VehicleStatus == VEHICLE_STOPPED) { // если автомобиль остановлен и готов для старта
		//обнулить промежуточные итоги
		this->MeasurmentStatus = MES_STOPPED; // статус измерения "остановлен"
	}
	//Проверка на начала замера с места
	//Если в текущем пакете ГПС появился курс, а в предыдущем он отсутствовал  и
	//средняя скорость предыдущих двух замеров меньше 1 км/ч
	if (gpsData.CourseTrue != 0 && (this->messageArray[3].CourseTrue == 0)
			&& (((this->messageArray[3].Speed + this->messageArray[2].Speed) / 2)
					< 1000)) {
		this->StartMeas = this->messageArray[3]; // Записываем точку старта
		this->VehicleStatus = VEHICLE_ACCELERATE; // статус автомобиля "Ускоряется"
		this->MeasurmentStatus = MES_ACCELERATE; // статус измерения "Ускоряется"

	}

	if (this->MeasurmentStatus == MES_ACCELERATE) {

	}
}
//проверяем переход через замеряемые величины скорости
void checkSpeedThrough(measurmentStruct *this) {
	for (uint8_t i = 0; i < 29; ++i) {
		if (this->gpsData.Speed > IntermediateMeasurementOfSpeed[i]
				&& this->messageArray[3].Speed
						< IntermediateMeasurementOfSpeed[i]) {
			uint16_t resTime = getResultTimeForSpeed(
					IntermediateMeasurementOfSpeed[i], this);
			//что то делаем
		}
	}
}

//Получить дорасчтное время замера
uint32_t getResultTimeForSpeed(uint16_t speed, measurmentStruct *this) {

	uint32_t resultTime = getDifTime(this->StartMeas.Time,
			this->messageArray[3].Time);
	float gpsDataSpeedF = this->gpsData.Speed;
	float messageArraySpeedF = this->messageArray[3].Speed;
	float koef = 1
			- ((gpsDataSpeedF - speed) / (gpsDataSpeedF - messageArraySpeedF));
	uint32_t difTime = (getDifTime(this->messageArray[3].Time,
			this->gpsData.Time)) * koef;
	resultTime += difTime;

	return resultTime;

}

