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

#ifdef FALSE_WIRE
#include "mpu_umulator.h"
#endif

//===================================================

//#define SESOR_UPSIDE_DOWN
class MpuClass 
{
	friend class HmcClass;
 protected:
	 void do_magic();
	 void calc_corrected_ang();
	 void log();

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


	 int r;
	 int initialized = 0;
	 int dmpReady = 0;
	 float lastval[3];
	 int16_t sensors;
	 Quaternion q;
	 uint8_t rate;
	 float gaccX, gaccY;
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
	//-----------------
	float m7_accX, m7_accY;
	float windFX, windFY;
	float e_speedX, e_speedY;
	
	float w_accX, w_accY;
	//------------------
	float yaw_off;
	float f_pitch, f_roll;
	float pitch, roll;

	void meansensors();
	void calibrationF(int16_t ar[]);
	void calibrationF0(int16_t ar[]);
	void calibrationPrint(int16_t ar[],const bool onlyGyro);
	float yaw,yaw_offset;
	uint64_t acc_callibr_time;
	float DRAG_K,_0007;
 public:
	 uint64_t mputime;
	float e_accX, e_accY;
	 float cor_c_pitch, cor_c_roll;
	 uint64_t oldmpuTime;


	 float cosYaw,sinYaw;
	 int8_t max_g_cnt;

	 float temp_deb;
	 float faccX,faccY,faccZ;
	 void initYaw(const float angle);
	
	 void new_calibration(const bool onlyGyro);
	 
	 float get_yaw();
	 float get_pitch();
	 float get_roll();
	 bool mpu_calibrated,gyro_calibratioan;
	float accZ,accY,accX,tiltPower,cosPitch,cosRoll,sinPitch,sinRoll;
	float tiltPower_CF;
	float maccX, maccZ, maccY;
	
	
	 float  gyroPitch, gyroYaw, gyroRoll;


	 string get_set();
	 void set(const float  *ar);


	 bool selfTest();
	 float dt,rdt;
	 void set_yaw();

	 int16_t getGX();


	void init();
	bool loop();
	void setDLPFMode_(const uint8_t f);
};

extern MpuClass Mpu;

#endif



