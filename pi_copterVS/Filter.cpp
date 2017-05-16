
#include "Filter.h"

void FilterClass::init(float t)
{
	T = 1.0f/t;
	tt=0;
	max = min = ret = 0;
//	bool rise = true;


}

float FilterClass::get(float v,float dt){
	if ((tt += dt) > T){
		ret = (max + min)*0.5f;
		tt = 0;
	}
	if (rise){
		if (v >= max)
			max = v;
		else{
			rise = false;
			ret = (max + min)*0.5f;
			min=max;
		}
	}
	else {
		if (v <= min)
			min = v;
		else{
			rise = true;
			ret = (max + min)*0.5f;
			max=min;
		}
	}
	return ret;

}

FilterClass Filter;

