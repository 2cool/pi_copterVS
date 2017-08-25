

#ifndef _LOG_h
#define _LOG_h

#include "define.h"
#include "Location.h"


/*
mpu=34b
gps=11b
hmc=17b
MS5=6b
comm=1024

*/
enum LOG { MPU, HMC, MS5, GpS, COMM, STABXY, STABZ, BAL, GPS_FULL };
class LogClass
{
private:
	
	
public:
	bool writeTelemetry;
	int counter = 0;
	int run_counter;
	void end();
	void loadGPS(NAV_POSLLH *gps);
	void loadGPS_full(NAV_POSLLH *gps);
	void loaduint32t(uint32_t ui);
	void loadFloat(float f);
	void loadInt16t(int16_t i);
	void loadMem(uint8_t*buf, int len);
	void loadByte(uint8_t b);
	bool init(int counter);
	bool close();
};

extern LogClass Log;

#endif