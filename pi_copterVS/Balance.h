// Balance.h

#ifndef _BALANCE_h
#define _BALANCE_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif
#include "Hmc.h"

#include "Pwm.h"
#include "mpu.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Autopilot.h"
#include "AP_PID.h"

/*
    ^
0   |    1
 \  |  /
  \   /
   000
   000
  /   \
 /     \
2       3

*/

class BalanceClass
{
 protected:
	 uint32_t power_on_time;
	float throttle;
	//float pitch_roll_stabKP, pitch_roll_rateKP, yaw_rateKP,yaw_stabKP,pitch_roll_rateKI,pitch_roll_rateIMAX,yaw_rateKI,yaw_rateIMAX;
	//float getThrottle();
	float yaw_stabKP, pitch_roll_stabKP;

	float f_[4];
float maxAngle;
 public:
	 bool power_is_on() { 
#ifdef NO_BATTERY
	    return true; 
#else 
		return (power_on_time > 0 && millis() - power_on_time > 2000);
#endif 
	 }

	 float powerK();
	 float c_pitch, c_roll;
	 float get_throttle(){ return throttle; }
	 float gf(const uint8_t n){ return f_[n]; }
	 float gf0(){ return f_[0]; }
	 float gf1(){ return f_[1]; }
	 float gf2(){ return f_[2]; }
	 float gf3(){ return f_[3]; }
	// float get_throttle(){ return (throttle-1000)*0.001; }

	// bool set_min_max_throttle(const float min, const float max);
	 
	 


void setMaxAngle(const float ang);

string get_set();
void set(const float  *ar);

	void init();
	void loop();



	void set_off_th_(){ f_[0] = f_[1] = f_[2] = f_[3] = 0; throttle = 0;}
	float pitch2rollK;
#define BALANCE_PIDS 3
	AP_PID pids[BALANCE_PIDS];
private:

#define PID_PITCH_RATE 0
#define PID_ROLL_RATE 1
#define PID_YAW_RATE 2



	float accXaccY_K;
	

};

extern BalanceClass Balance;

#endif

