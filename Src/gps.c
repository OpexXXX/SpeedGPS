/*
 * gps.c
 *
 *  Created on: 21 июн. 2018 г.
 *      Author: opex
 */
#include "gps.h"
#include "ledDriver.h"

char Time[12]=""; //время
char Status[2]=""; //валидность
char SLatitude[16]="";  //Латитуда
char NS[3]="";                          //
char SLongitude[12]="";         //Лонгитуда
char EW[3]="";                          //
char CourseTrue[10]="";                 // курс
char Data[12]="";                               //Дата
char SatCount[4]="";                    //используемых спутников
char AltitudaMSL[12]="";            //высота
char ViewSat[4];   //
char COG[8]="";                 //
char COGstat[4]="";             //
char Speed[8]="";                       //скорость
char SpeedAlt[8]="";    //
char UNUSED[32]="";                     //мусорка, тут все данные, которые не нужны
char Knot[8]="";
char *const RMC[]={Time,Status,SLatitude,NS,SLongitude,EW,Speed,CourseTrue,Data,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED};
char *const GGA[]={UNUSED,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED,SatCount,UNUSED,AltitudaMSL,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED};
char *const GSV[]={UNUSED,UNUSED,ViewSat,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED};
char *const VTG[]={COG,COGstat, UNUSED,UNUSED,Knot,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED};
unsigned char GLONAS_COUNT=0;
unsigned char GPS_COUNT=0 ;
volatile char DataDone=0 ;
unsigned char DataValid=0;

extern  osMessageQId GPSHandlerHandle;







int AsciiToInt(char* s)
{
	int n = 0;
	while( *s >= '0' && *s <= '9' ) {
		n *= 10;
		n += *s++;
		n -= '0';
	}
	return n;
}
uint32_t AsciiBeforeDotToInt(char* s)
{
	uint32_t n = 0;
	while( *s >= '0' && *s <= '9' ) {
		if(*s =='.')  return n;
		n *= 10;
		n += *s++;
		n -= '0';
	}
	return n;
}
uint32_t AsciiAfterDotToInt(char* s)
{
	uint32_t n = 0;
	uint8_t flagDot=0;
	while( *s ) {
		if(*s =='.')
		{
			flagDot=1;
			s++;
		}
		if(flagDot && (*s) ){
			n *= 10;
			n += *s++;
			n -= '0';
		}else{
			s++;
		}

	}
	return n;
}
uint32_t AsciiRemoveDotToInt(char* s)

{
	uint32_t n = 0;
	while( *s ) {
		if(*s =='.')
		{

			s++;
		}
		if(*s){
			n *= 10;
			n += *s++;
			n -= '0';
		}else{
			s++;
		}

	}
	return n;
}

void uartParserGps(unsigned char data)
{

	portBASE_TYPE xStatus;

	switch (Parser(data)) {
		case GPS_NRMC:
			if(Status[0]='A'){
							gpsSpeedMessegeStruct gpsUInt;
							gpsUInt.Time = AsciiRemoveDotToInt(&Time)*10;
							float tempSpeed = AsciiRemoveDotToInt(&Speed);
							tempSpeed = tempSpeed*1.852;
							gpsUInt.Speed =  tempSpeed;
							gpsUInt.SLatitude =  AsciiRemoveDotToInt(&SLatitude);
							memcpy(gpsUInt.NS, NS, sizeof(NS));
							memcpy(gpsUInt.EW, EW, sizeof(EW));
							gpsUInt.SLongitude =  AsciiRemoveDotToInt(&SLongitude);
							gpsUInt.CourseTrue =  AsciiRemoveDotToInt(&CourseTrue);
							xStatus = xQueueSendToBack(GPSHandlerHandle,&gpsUInt,0);
			}
			break;
		default:
			break;
	}



}
uint32_t getDifTime(uint32_t startTime, uint32_t stopTime )
{
	 uint64_t result=0;
	int64_t hour=0;
	int64_t minute=0;
	int64_t seconds = 0;
//150012300
	int64_t difHour= (stopTime /10000000)-(startTime /10000000);

	int64_t difMinute =  (stopTime/100000)%100+difHour*60 -(startTime/100000)%100 ;

	int64_t difSecons =    (stopTime%100000)+difMinute*60000-(startTime%100000) ;

	result = difSecons;
	    return result;

}
uint8_t Parser(unsigned char data)
{
	static unsigned char ByteCount=0xff;
	static unsigned int MsgType;
	static char *MsgTxt=(char*)&MsgType;
	static  unsigned char ComaPoint=0xff;
	static unsigned char CharPoint=0;
	if(data=='$'){ByteCount=0;ComaPoint=0xff;MsgTxt=(char*)&MsgType; return 0;} //ждем начала стрки
	if(ByteCount==0xff) return 0;                                                                     //
	ByteCount++;
	if(ByteCount<=1)        return 0;                                                         //
	if(ByteCount<6&&ByteCount>1)            //берем 4 символа заголовка
	{
		*MsgTxt=data;   //и делаем из него число
		MsgTxt++;
		return 0;
	}
	//
	switch(MsgType)
	{
	case    0x434D5250:                             //GPRMC
	case    0x434D524E:                             //GNRMC
		if(data==',') {ComaPoint++;     CharPoint=0;RMC[ComaPoint][0]=0;return 0;}
		if((data)==('*')) {MsgType=0;

		return GPS_NRMC;}
		RMC[ComaPoint][CharPoint++]=data;
		RMC[ComaPoint][CharPoint]=0;
		return 0;
	case    0x41474750:                             //PGGA
	case    0x4147474e:                             //NGGA
		if(data==',')  {ComaPoint++;    CharPoint=0;GGA[ComaPoint][0]=0;return 0;}
		if(data=='*') {MsgType=0;return GPS_NGGA;}
		GGA[ComaPoint][CharPoint++]=data;
		GGA[ComaPoint][CharPoint]=0;
		return 0;
	case    0x47545650:             //PVTG
		if(data==',')  {ComaPoint++;    CharPoint=0;VTG[ComaPoint][0]=0;return 0;}
		if(data=='*') {
			return GPS_PVTG;
		}
		VTG[ComaPoint][CharPoint++]=data;
		VTG[ComaPoint][CharPoint]=0;
		return 0;
	case    0x4754564e:             //NVTG
		if(data==',')  {ComaPoint++;    CharPoint=0;VTG[ComaPoint][0]=0;return 0;}
		if(data=='*') {
			return GPS_NVTG;}
		VTG[ComaPoint][CharPoint++]=data;
		VTG[ComaPoint][CharPoint]=0;
		return 0;
	case    0x56534750:             //PGSV
		if(data==',')  {ComaPoint++;    CharPoint=0;GSV[ComaPoint][0]=0;return 0;}
		if(data=='*')  {GPS_COUNT=AsciiToInt(ViewSat);MsgType=0;return GPS_PGSV;}
		GSV[ComaPoint][CharPoint++]=data;
		GSV[ComaPoint][CharPoint]=0;
		return 0;
	case    0x5653474c:             //LGSV
		if(data==',')  {ComaPoint++;    CharPoint=0;GSV[ComaPoint][0]=0;return 0;}
		if(data=='*') {GLONAS_COUNT=AsciiToInt(ViewSat);MsgType=0;return GPS_LGSV;}
		GSV[ComaPoint][CharPoint++]=data;
		GSV[ComaPoint][CharPoint]=0;
		return 0;
	default:        ByteCount=0xff;break;
	}
	ByteCount=0xff;
	return 0;
}


