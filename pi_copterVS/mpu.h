#include "define.h"

// mpu.h

#ifndef _MPU_h
#define _MPU_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif
#include "Filter.h"
#include "I2Cdev.h"
#include "MPU6050.h"

#include "RC_Filter.h"
#include "helper_3dmath.h"
//#include "MotionSensor.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"


#include "define.h"

//===================================================

//#define SESOR_UPSIDE_DOWN
class MpuClass 
{
	friend class HmcClass;
 protected:
	 float roll_max_angle, pitch_max_angle;
	 float add_2_yaw;
	 int ms_open();
	 // MPU control/status vars
	 uint8_t devStatus;      // return status after each device operation
							 //(0 = success, !0 = error)
	 uint8_t fifoCount;     // count of all bytes currently in FIFO
	 uint8_t fifoBuffer[64]; // FIFO storage buffer

	 int16_t a[3];              // [x, y, z]            accel vector
	 int16_t g[3];              // [x, y, z]            gyro vector
	 int32_t _q[4];
	 int32_t t;
	 int16_t c[3];

	 VectorFloat gravity;    // [x, y, z]            gravity vector

	 int r;
	 int initialized = 0;
	 int dmpReady = 0;
	 float lastval[3];
	 int16_t sensors;
	 Quaternion q;
	 uint8_t rate;

	 //calibration offsets for MPU6050
	 ///////////////////////////////////   CONFIGURATION   /////////////////////////////
	 //Change this 3 variables if you want to fine tune the skecth to your needs.
	 const int buffersize = 1000;     //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
	 const int acel_deadzone = 8;     //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
	 const int giro_deadzone = 1;     //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)
	 
	 int mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz;
	 enum{ ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset };
	// int16_t offset_[6];
	 ////////////////////////////////////////////////////////////////////////////////////////
	 float fx, fy, fz;
	// float const_gyroRoll0, const_gyroPitch0, const_gyroYaw0;
	 uint32_t gyroTime;
	 
	 
	MPU6050 accelgyro;
	 float h_yaw;
	uint8_t gLPF;


	  uint64_t oldmpuTime;
	  float pitch, roll;
	  void meansensors();
	  void calibrationF(int16_t ar[]);
	  void calibrationF0(int16_t ar[]);
	  void calibrationPrint(int16_t ar[],const bool onlyGyro);
	  float cS;
	  float speedX, speedY;
	  void set_max_angle(float cx, float cy);
	  uint64_t acc_callibr_time;
 public:
	 float get_roll_max_a() { return roll_max_angle; }
	 float get_pitch_max_a() { return pitch_max_angle; }
	 void set_cS(const float v){ cS = v; }
	 float cosYaw,sinYaw;
	 int8_t max_g_cnt;
#ifdef FALSE_MPU
	 float wind_x,wind_y;
	 float windX();
	 float windY();
#endif
	 bool upsidedown;
	float altitude_Z, speed_Z;
#ifdef FALSE_MPU
	float fspeedX, fspeedY, fdistX, fdistY;
	 void getFalse(float &accx, float &accy);
	 float getZA();
#endif


	 float temp_deb;
	 float faccX,faccY,faccZ;
	 void initYaw(const float angle);
	
	 void new_calibration(const bool onlyGyro);
	 
	 
	 float get_pitch(){ return pitch; }
	 float get_roll(){ return roll; }
	 bool mpu_calibrated,gyro_calibratioan;
	float accZ,accY,accX,tiltPower,cosPitch,cosRoll,sinPitch,sinRoll;
	
	
	 float yaw,  gyroPitch, gyroYaw, gyroRoll;


	 string get_set();
	 void set(const float  *ar);


	 bool selfTest();
	 float dt,rdt;
	 void set_yaw();

	 int16_t getGX();


	void init();
	void loop();
	void setDLPFMode_(const uint8_t f);
};

extern MpuClass Mpu;

#endif



