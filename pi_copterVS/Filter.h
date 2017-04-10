// Filter.h
#ifndef _FILTER_h
#define _FILTER_h



class FilterClass
{
 protected:
	 float max, min, ret,T,tt;
	 bool rise;
	
 public:
	void init(float t); 
	float get(float v,float dt);
};

extern FilterClass Filter;

#endif

