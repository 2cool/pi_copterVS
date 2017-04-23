#ifndef MS5611_h
#define MS5611_h

#include <stdint.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/types.h>
#include <fcntl.h>
#include <math.h>
#include <time.h>

#include "WProgram.h"
#include "define.h"

#include "FilterTwoPole.h"
#define MS5611_ADDRESS 0x77

#define CONV_D1_256   0x40
#define CONV_D1_512   0x42
#define CONV_D1_1024  0x44
#define CONV_D1_2048  0x46
#define CONV_D1_4096  0x48
#define CONV_D2_256   0x50
#define CONV_D2_512   0x52
#define CONV_D2_1024  0x54
#define CONV_D2_2048  0x56
#define CONV_D2_4096  0x58
#define CMD_ADC_READ  0x00
#define CMD_PROM_READ 0xA0

#define OSR_256      1000 //us
#define OSR_512      2000 //us
#define OSR_1024     3000 //us
#define OSR_2048     5000 //us
#define OSR_4096     10000 //us

#define alpha 0.96
#define beta 0.96
#define gamma 0.96

// check daily sea level pressure at 
// http://www.kma.go.kr/weather/observation/currentweather.jsp
#define SEA_LEVEL_PRESSURE 1023.20 // Seoul 1023.20hPa

class MS5611Class {
	
protected:
	 int ct;
	 float presure_altitude_at_start;
	 float altitude_error;

	 inline void phase0();
	 inline void phase1();
	 inline void phase2();
	 inline bool phase3();
	 inline void phase4();

	float altitude_;
public:
	 float altitude(){ return altitude_; }

	uint8_t ms5611_count;
	int init();
	uint8_t loop();
	uint32_t lastTime;
	float pressure , powerK;


	void copterStarted();

	int8_t i_readTemperature;
	
	float speed;
	


	double getAltitude(const float pres);

	float get_pressure(float h);
	//---------------------------------------------------------------



private:


	int i;
	int fd;
	uint16_t C[7];
	uint32_t D1;
	uint32_t D2;
	
	int64_t dT;
	int32_t TEMP;
	int64_t OFF;
	int64_t SENS;
	int32_t P;

	int bar_task;
	uint32_t b_timeDelay;
	uint8_t bar_D[3];
	int  bar_h;
	char bar_zero;

	long curSampled_time;
	long prevSampled_time;
	float Sampling_time, prevSampling_time;
	

	
public:
	double temparature;

	void update();
	
};

extern MS5611Class MS5611;

#endif
