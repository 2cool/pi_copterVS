// commander.h

#ifndef _COMMANDER_h
#define _COMMANDER_h




#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif
#include "Autopilot.h"
class CommanderClass
{
 protected:
	 volatile int data_size;
	 uint8_t buf[TELEMETRY_BUF_SIZE];

//	 bool ManualControl(int8_t *msg);//manual

//	 bool ButtonMessage(byte *msg);//button command

	 bool ButtonMessage(string);

	 float out_yaw;
	 float throttle, yaw, yaw_offset, pitch, roll;
	 bool Settings(string msg);//settings

 public:
	 uint8_t _set(const float  val, float &set, bool secure=true);
	 void setThrottle(const float t){ throttle = t; }
	 void setPitch(const float p){ pitch = p; }
	 void setRoll(const float r){ roll = r; }
	 float getYaw(){ return out_yaw; }
	 float getThrottle(){ return throttle; }
	 float getPitch();
	 float getRoll();
	// float getYaw_offset(){ return yaw_offset; }
	 float get_contr_yaw(){ return yaw; }
	 bool ret;
	//short recived_counter;
	
	

	 void init();
	 bool input();
	 void new_data(byte *buf, int n);
};
	

extern CommanderClass Commander;

#endif

