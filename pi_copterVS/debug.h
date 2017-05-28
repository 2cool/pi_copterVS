// debug.h

#ifndef _DEBUG_h
#define _DEBUG_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "mpu.h"

class DebugClass
{
 protected:
#define MAXARR 10
	 int i;
	 float ar[MAXARR][2];


	 void graphic(const int n, const float x, const float y);

 public:
	 int n_debug, n_p1, n_p2;
	 void init();
	 void dump(const long f1, long f2, long f3, long f4);
	 void dump(const float f1, float f2, float f3, float f4);
	 void dump(const uint8_t f1, uint8_t f2, uint8_t f3, uint8_t f4);
	 void load(const uint8_t i, const float x, const float y);
	 void dump();
};

extern DebugClass Debug;

#endif

