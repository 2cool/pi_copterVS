// Autopilot.h

#ifndef _AUTOPILOT_h
#define _AUTOPILOT_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include "mi2c.h"
#include "MS5611.h"
#include "Location.h"
#include "AP_PID.h"


// in dicimetr
#define MAX_HEIGHT 15


#include "commander.h"
#include "mpu.h"
class AutopilotClass
{



 protected:
#ifdef FALL_IF_STRONG_WIND
	 float dist2home_at_begin2;
#endif
	 bool howeAt2HOME;
	 float sens_z, sens_xy;

	 bool newData;
	 uint32_t controlDeltaTime;
	 uint8_t go2homeIndex;
	 float f_go2homeTimer;
	 uint32_t oldtime;
	 
	 
	 float flyAtAltitude;

	// bool motors_on, smart_ctrl;

	 float aPitch, aRoll,aYaw_;

	 uint32_t old_control_bits;
	 float throttle;
	 
	 uint32_t control_bits;
	 float tflyAtAltitude;

	 void log();

	 void smart_commander(const float dt);
	// void setNextGeoDot();
	 //void prog_loop();
	// float get_dist();
	 float lowest_height;
	
	enum {MOTORS_ON=1,CONTROL_FALLING=2,Z_STAB=4,XY_STAB=8,GO2HOME=0x10,PROGRAM=0x20,COMPASS_ON=0x40,HORIZONT_ON=0x80,
		MPU_ACC_CALIBR=0x100, MPU_GYRO_CALIBR = 0x200, COMPASS_CALIBR=0x400, COMPASS_MOTOR_CALIBR=0x800, SHUTDOWN=0x1000, GIMBAL_PLUS=0x2000,GIMBAL_MINUS=0x4000,REBOOT=0x8000
	};

	 
 public:

	 bool busy() { return (control_bits & (MPU_ACC_CALIBR | MPU_GYRO_CALIBR | COMPASS_CALIBR)); }
	 uint32_t last_time_data_recived;
	 void setYaw(const float yaw){aYaw_ = yaw;}
	 float getGimbalPitch(){ return gimbalPitch; }
	 void control_falling(const string msg);
	 void gimBalPitchADD(const float add);
	 void gimBalRollCorrection();
	 float corectedAltitude4tel();
	
	 uint32_t start_time;
	void reset_compas_motors_calibr_bit() {control_bits &= (~COMPASS_MOTOR_CALIBR);}

	bool motors_onState(){ return control_bits&MOTORS_ON; }
	bool z_stabState(){ return control_bits&Z_STAB; }
	bool xy_stabState(){ return control_bits&XY_STAB; }
	 bool go2homeState(){ return control_bits&GO2HOME; }
	 bool progState(){ return control_bits&PROGRAM; }
	 bool control_fallingState(){ return control_bits & CONTROL_FALLING; }
	 bool compass_onState(){ return control_bits&COMPASS_ON; }
	 bool horizont_onState(){ return control_bits&HORIZONT_ON; }
	 bool set_control_bits(uint32_t bits);
	 void compass_tr();
	 void horizont_tr();
	 float fly_at_altitude() { return flyAtAltitude; }
	 uint32_t get_control_bits(){ return control_bits; }
	 //uint8_t mod;  //режим работы 
	// bool falling(){ return ctrl_flag == CNTR_FALLING; }


	 float get_throttle(){ return throttle; }
	 float get_yaw(){ return aYaw_; }

	 
	 void add_2_need_altitude(float speed, const float dt);
	 void add_2_need_yaw(float speed, const float dt);
	// bool manualZ;
	
	string get_set();
	 void set(const float buf[]);

	
	 void clearSpeedCoreection(){ flyAtAltitude = tflyAtAltitude; }

	// bool get_smart_cntr_flag(){ return smart_ctrl; }
	 bool connected;
	 float gimBalPitchZero, gimBalRollZero, gimbalPitch;
	 float height_to_lift_to_fly_to_home;

	 bool going2HomeON(const bool hower);
	 bool going2HomeStartStop(const bool hower);
	 bool go2HomeProc(const float dt);

	 bool selfTest();
	 float get_Roll(){ return aRoll; }
	 float get_Pitch(){ return aPitch; }


	 
	 bool holdLocationStartStop();
	 bool holdLocation(const long lat, const long lon);
	 
	 bool holdAltitudeStartStop();
	 bool holdAltitude(float alt);
	 void	 set_new_altitude(float alt);
	 void init();
	 void loop();
	// uint8_t ctrl_flag;
	 bool motors_is_on(){ return control_bits & MOTORS_ON; }

	 bool start_stop_program(const bool stopHere);

	void connectionLost_();


	void calibration(); 

	//void set_height(const short h);
	bool off_throttle(const bool force,const string msg);
	bool motors_do_on(const bool start,const string msg);



private:

};

extern AutopilotClass Autopilot;

#endif

