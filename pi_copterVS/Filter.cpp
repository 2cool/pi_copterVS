
#include "Filter.h"

MYfILTER::MYfILTER()
{
	acum = 0;
	old_val = 0;
}


MYfILTER::~MYfILTER()
{
}
MYfILTER::MYfILTER(float f) {
	max_delta = f;
}


float MYfILTER::update(const float val) {
	float delta = val - old_val;
	float max_d = max_delta;// acum*0.01;
	/*
	if (max_d < max_delta)
	max_d = max_delta;
	else if (max_d < -max_delta)
	max_d = -max_delta;
	*/
	//if (max_d < 0)
	//	max_d = -max_d;


	if (delta > max_d) {
		acum += delta - max_d;
		old_val += (max_d);
	}
	else if (delta < -max_d) {
		acum += (delta + max_d);
		old_val -= max_d;
	}
	else
		old_val = val;
	
	if (acum > max_delta) {
	acum -= max_delta;
	old_val += max_delta;
	}
	else if (acum < -max_delta) {
	acum += max_delta;
	old_val -= max_delta;
	}
	

	return old_val;// *0.9;
}

