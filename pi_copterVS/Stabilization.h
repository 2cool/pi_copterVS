// Stabilization.h

#ifndef _STABILIZATION_h
#define _STABILIZATION_h


#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif



#include "AP_PID.h"
#include "define.h"
class StabilizationClass{


#define ACCX_SPEED 0
#define ACCY_SPEED 1
#define ACCZ_SPEED 2

private:
	

	float accXY_stabKP, accXY_stabKP_Rep;
	float accZ_stabKP, accZ_stabKP_Rep;
	//float throttle;
#define STAB_PIDS 3
	AP_PID pids[STAB_PIDS];
	float max_z_integrator;
	float  sX, sY, speedX, speedY;
	//unsigned long gps_sec;
	float speedZ, sZ;
	float Z_CF_SPEED,Z_CF_DIST;

	void set_acc_xy_speed_kp(const float f){ pids[ACCX_SPEED].kP(f);	pids[ACCY_SPEED].kP(f); }
	void set_acc_xy_speed_kI(const float f){ pids[ACCX_SPEED].kI(f);	pids[ACCY_SPEED].kI(f); }
	void set_acc_xy_speed_kD(const float f){ pids[ACCX_SPEED].kD(f,3);	pids[ACCY_SPEED].kD(f,3); }
	void set_acc_xy_speed_imax(const float f){ pids[ACCX_SPEED].imax(f);	pids[ACCY_SPEED].imax(f); }
	void  resset_xy();
	void  resset_z();
public:

	float getAltitude() { return sZ; }
	float getSpeed_Z(float dist){
		return dist*accZ_stabKP;
	}

	float getDist_Z(float speed){
		return speed*accZ_stabKP_Rep;
	}

	float getSpeed_XY(float dist){		
		return dist*accXY_stabKP;
	}
	float getDist_XY(float speed){	
		return speed*accXY_stabKP_Rep;
	}



	float getSpeedX(){ return speedX; }
	float getSpeedY(){ return speedY; }
	float getDistX(){ return sX; }
	float getDistY(){ return sY; }
	void setDefaultMaxSpeeds();
	
	//float get_accZ_stabKP_Rep(){ return accZ_stabKP_Rep; }
//	float getDist2SpeedXYKP_Rep(){ return accXY_stabKP_Rep; }
//	float getDist2SpeedXYKP(){ return accXY_stabKP; }
	long get_sX_mul_100(){return (long)(sX * 100); 	}
	long get_sY_mul_100(){ return (long)(sY * 100); }
	void init_XY(const float sx, const float speedx, const float sy, const float speedy);
	void init_Z();
	float XY_KF_SPEED, XY_KF_DIST;
	
	

	float max_stab_z_P,max_stab_z_M,max_speed_xy;
	float last_accZ;
	float Z();
	void XY(float &xF, float&yF);
	void horizont(const uint8_t i, const float dt, const float accX, const float accY, const float F);

	void init();
	string get_z_set();
	string get_xy_set();
	void setZ(const float  *ar);
	void setXY(const float  *ar);


};
extern StabilizationClass Stabilization;
#endif

