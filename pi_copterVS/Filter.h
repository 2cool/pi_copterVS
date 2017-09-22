// Filter.h
#ifndef _FILTER_h
#define _FILTER_h


class MYfILTER
{

private:
	float max_delta = 0.15;
	float acum = 0;
	float old_val = 0;
public:
	void setMax(float val) { max_delta = val; }
	float update(float val);
	MYfILTER(float f);
	MYfILTER();
	~MYfILTER();
};

#endif

