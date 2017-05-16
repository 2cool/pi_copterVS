// RC_Filter.h

#ifndef _RC_FILTER_h
#define _RC_FILTER_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif
class RC_filter{
public:
	void reset(){ last_v = 0; }
	void setF(const float _fCut, float lv=0){
		RC = 1.0f / (2.0f * (float)M_PI*_fCut);
		last_v = lv;
	}
	float get(float v, const float delta_time){
		v = last_v + (delta_time / (RC + delta_time)) * (v - last_v);
		last_v = v;
		return v;
	}
private:
	float RC,last_v;
};

#endif

