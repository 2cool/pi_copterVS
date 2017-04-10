// Ultrasound_Radar.h

#ifndef _ULTRASOUND_RADAR_h
#define _ULTRASOUND_RADAR_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif
#define UNKNOWN_VALUE 10
class Ultrasound_RadarClass
{
 protected:

 public:
	 boolean motors_energized;
	 float dist2ground_;
	void loop();
		void init();
		void reset();
};

extern Ultrasound_RadarClass Ultrasound_Radar;

#endif

