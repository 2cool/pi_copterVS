// LED.h



#ifndef _LED_h
#define _LED_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "Autopilot.h"
class LEDClass
{

	enum { MOTORS_ON = 1, CONTROL_FALLING = 2, Z_STAB = 4, XY_STAB = 8, GO2HOME = 16, COMPASS_ON = 32, HORIZONT_ON = 64 };
 protected:

	 
	 uint16_t c = 0;


	 uint8_t prog(const uint8_t shift, const uint16_t mask);
	 

 public:
	 void off();
	 enum{ MOT_OFF_P, MANUAL_P, SEMI_SMART_P, FULL_SMART_P, GO_TO_HOME_P, PROG0_P, LED_OFF,PROG1_P };
	 void light(const int n);
	 uint8_t prog_index;
	 bool light_on;
	void init();
	void loop();
	void buzz();
	uint32_t error_time;
	uint8_t error_code;
};

extern LEDClass LED;

#endif

